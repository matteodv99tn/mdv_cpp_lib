#ifndef MDV_DMP_HPP
#define MDV_DMP_HPP

#include <cmath>
#include <cstdint>
#include <gsl/assert>
#include <spdlog/spdlog.h>

#include "mdv/containers/demonstration.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/utils/conversions.hpp"
#include "mdv/utils/logging.hpp"

#define SQUARE(x) ((x) * (x))

namespace mdv {

template <typename T>
struct type_elems_size;

template <typename T>
constexpr std::size_t type_elems_size_v = type_elems_size<T>::value;

template <int EigSize>
struct type_elems_size<Eigen::Matrix<double, EigSize, 1>> {
    static_assert(EigSize > 0);
    static constexpr std::size_t value = EigSize;
};

template <>
struct type_elems_size<double> {
    static constexpr std::size_t value = 1;
};

template <riemann::manifold M>
class Dmp {
public:
    MDV_MANIFOLD_TYPENAMES_IMPORT(M);

    using basis_size_t = unsigned long;
    using weights_t    = Eigen::MatrixXd;

    Dmp(const double       alpha   = 48.0,
        const double       beta    = 12.0,
        const double       gamma   = 3.0,
        const basis_size_t n_basis = 12) :
            _alpha(alpha), _beta(beta), _gamma(gamma), _n_basis(n_basis) {
        construct_basis_parameters();

        logger().info("Initialised DMP object");
        logger().debug("  alpha = {}", _alpha);
        logger().debug("  beta  = {}", _beta);
        logger().debug("  gamma = {}", _gamma);
        logger().debug("  number of basis: {}", _n_basis);

        _ws = Eigen::MatrixXd::Zero(_n_basis, type_elems_size_v<TangentVector>);
    }

    template <typename Demonstration>
    MDV_NODISCARD Eigen::MatrixXd
                  evaluate_desired_forcing_term(const Demonstration& demo) {
        using mdv::convert::seconds;
        static constexpr bool is_scalar = type_elems_size_v<TangentVector> == 1;

        Eigen::MatrixXd f_des(demo.size(), type_elems_size_v<TangentVector>);
        const auto&     g = demo.back().y();
        for (long i = 0; i < demo.size(); ++i) {
            const auto&  sample  = demo[i];
            const auto&  pos_err = M::logarithmic_map(sample.y(), g);
            const auto&  vel_err = sample.yd();
            const auto&  acc_err = sample.ydd();
            const double s       = time_to_s(seconds(sample.t()));
            assert(s > 1e-6);
            auto res = tau * tau * acc_err - _alpha * (_beta * pos_err - tau * vel_err);
            if constexpr (is_scalar) {
                f_des(i) = res;
            } else {
                f_des.row(i) = res;
            }
        }
        assert(!f_des.hasNaN());
        return f_des;
    }

    template <typename Demonstration>
    void
    learn(const Demonstration& demo) {
        using mdv::convert::seconds;
        tau = seconds(demo.back().t() - demo.front().t());

        logger().info("Training DMP on a demonstration with {} samples", demo.size());
#ifdef MDV_VERBOSE_DMP
        logger().info("  tau = {}s", tau);
        logger().info("  dt  = {}s", mdv::convert::seconds(demo[1].t()));
        logger().info("  y0  = {}", demo.front().y());
        logger().info("  g   = {}", demo.back().y());
#endif

        logger().trace("Evaluating desired forcing term");
        const Eigen::MatrixXd f_des = evaluate_desired_forcing_term(demo);

        logger().trace("Evaluating matrix Phi");
        Eigen::MatrixXd phi(demo.size(), _n_basis);
        for (auto i = 0; i < demo.size(); ++i)
            phi.row(i) = eval_basis(time_to_s(demo[i].t())) * time_to_s(demo[i].t());

        assert(phi.rows() == demo.size());
        assert(phi.cols() == _n_basis);
        assert(f_des.rows() == demo.size());
        assert(f_des.cols() == type_elems_size_v<TangentVector>);
        _ws = phi.fullPivHouseholderQr().solve(f_des);
        logger().info("DMP succesfully learned");


        const Eigen::MatrixXd ae = (phi * _ws - f_des).cwiseAbs();
        logger().info("Linear regression mean absolute error: {}", ae.mean());
        logger().info("Linear regression maximum absolute error: {}", ae.maxCoeff());

#ifdef MDV_VERBOSE_DMP
        logger().info("Integrating learned dynamics to compare learning outcome...");
        const auto reconstructed_demo =
                integrate(demo.front().y(), demo.back().y(), demo.size(), demo[1].t());

        Eigen::MatrixXd pos_err(f_des.rows(), f_des.cols());
        Eigen::VectorXd pos_err_norm(f_des.rows());
        for (auto i = 0; i < demo.size(); ++i) {
            if constexpr (std::is_scalar_v<Point>) {
                pos_err(i, 0) =
                        M::logarithmic_map(demo[i].y(), reconstructed_demo[i].y());
                // pos_err_norm(i) = std::abs(pos_err(i, 0));
            } else {
                pos_err.row(i) =
                        M::logarithmic_map(demo[i].y(), reconstructed_demo[i].y());
            }
            pos_err_norm(i) = pos_err.row(i).norm();
        }
        logger().info("Position error (norm in the tangent space):");
        logger().info("  mean: {}", double(pos_err_norm.mean()));
        logger().info("  min:  {}", double(pos_err_norm.minCoeff()));
        logger().info("  max:  {}", double(pos_err_norm.maxCoeff()));
#endif
    }

    Demonstration<M>
    integrate(
            const Point&                 y0,
            const Point&                 g,
            std::size_t                  n_steps,
            const Demonstration<M>::Time dt
    ) const {
        using mdv::condition::are_orthogonal;
        using mdv::convert::seconds;
        logger().info("Performing integration");
#ifdef MDV_VERBOSE_DMP
        logger().info("  y0: {}", y0);
        logger().info("  g:  {}", g);
        logger().info("  dt: {}s", mdv::convert::seconds(dt));
        logger().info("  number of steps: {}", n_steps);
#endif

        auto res = Demonstration<M>::empty(n_steps);
        res.append_sample()
                .position(y0)
                .velocity(M::default_tangent_vector())
                .acceleration(M::default_tangent_vector())
                .time(Demonstration<M>::Time::zero());
        Point         y     = y0;
        Point         yprev = y;
        TangentVector z     = M::default_tangent_vector();

        const auto dts = seconds(dt);
        for (auto i = 1; i < n_steps; ++i) {
            const double        s       = time_to_s(i * dt);
            const TangentVector f       = eval_weighted_basis(s) * s;
            const TangentVector log_y_g = M::logarithmic_map(y, g);

            const TangentVector dz_dt =
                    M::covariant_derivative(y, _alpha * (_beta * log_y_g - z) + f);
            const TangentVector z_next = z + dz_dt * dts / tau;

            y = M::exponential_map(yprev, z * dts / tau);
            z = M::parallel_transport(yprev, y, z_next);

            res.append_sample().time(i * dt).position(y).velocity(z / tau).acceleration(
                    dz_dt / tau
            );
            yprev = y;
        }
        return res;
    }

    TangentVector
    eval_weighted_basis(const double s) const {
        const Eigen::VectorXd b = eval_basis(s);

        if constexpr (type_elems_size_v<TangentVector> == 1) {
            return double((b.transpose() * _ws).value());
        } else {
            TangentVector res = b.transpose() * _ws;
            return res;
        }
    }

    double tau;

    MDV_NODISCARD double
    time_to_s(const double t) const {
        return std::exp(-_gamma / tau * t);
    }

    template <typename Duration>
    MDV_NODISCARD double
    time_to_s(const Duration t) const {
        return time_to_s(mdv::convert::seconds(t));
    }

    Eigen::VectorXd
    eval_basis(const double s) const {
        assert(_basis_c.rows() == _n_basis);
        assert(_basis_h.rows() == _n_basis);

        Eigen::VectorXd basis(_n_basis);
        for (auto i = 0; i < _n_basis; ++i)
            basis(i) = std::exp(-_basis_h[i] * SQUARE(s - _basis_c[i]));
        basis /= basis.sum();  // Normalise
        return basis;
    }

    MDV_NODISCARD std::size_t
                  n_basis() const {
        return _n_basis;
    }

    // clang-format off
    MDV_NODISCARD const Eigen::MatrixXd weights() const { return _ws; }

    // clang-format on


private:
    // Transformation - Coordinate System
    double _alpha;
    double _beta;
    double _gamma;

    // Basis
    basis_size_t    _n_basis;
    Eigen::VectorXd _basis_c;
    Eigen::VectorXd _basis_h;
    weights_t       _ws;

    void
    construct_basis_parameters() {
        logger().debug("Constructing parameters for a basis of size {}", _n_basis);
        _basis_c = Eigen::VectorXd(_n_basis);
        _basis_h = Eigen::VectorXd(_n_basis);

        for (auto i = 0; i < _n_basis; ++i) {
            _basis_c(i) = std::exp(-_gamma * i / _n_basis);
            logger().trace("c[{}] = {}", i, _basis_c(i));
        }

        for (auto i = 0; i < _n_basis - 1; ++i)
            _basis_h(i) = 1 / SQUARE(_basis_c(i + 1) - _basis_c(i));
        _basis_h(_n_basis - 1) = _basis_h(_n_basis - 2);
        for (auto i = 0; i < _n_basis; ++i)
            logger().trace("h[{}] = {}", i, _basis_h(i));
    }

    mutable SpdLoggerPtr _logger =
            mdv::static_logger_factory("dmp", mdv::LogLevel::Debug);

    spdlog::logger&
    logger() const {
        return *(_logger.get());
    }
};

}  // namespace mdv

#undef SQUARE

#endif  // MDV_DMP_HPP
