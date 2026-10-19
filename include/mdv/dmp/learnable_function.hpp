#ifndef MDV_DMP_LEARNABLE_FUNCTION_HPP
#define MDV_DMP_LEARNABLE_FUNCTION_HPP

#include <cassert>
#include <cmath>
#include <cstddef>
#include <Eigen/Dense>
#include <iterator>
#include <type_traits>
#include <vector>

#include "mdv/macros.hpp"
#include "mdv/utils/eigen_solvers.hpp"

#define SQUARE(x) ((x) * (x))

namespace mdv::dmp {

class ExponentialBasis {
public:
    using Input  = double;
    using Vector = std::vector<ExponentialBasis>;

    static Vector
    create_from_centers(const Eigen::VectorXd& cs) {
        // cs: set of desired base centers
        Vector basis(static_cast<std::size_t>(cs.size()));
        for (Eigen::Index i = 0; i < cs.size(); ++i) {
            const auto k = static_cast<std::size_t>(i);
            basis[k]._c  = cs(i);
            if (i < cs.size() - 1) basis[k]._h = 1 / SQUARE(cs(i + 1) - cs(i));
            else basis[k]._h = basis[k - 1]._h;
        }

        return basis;
    }

    ExponentialBasis(const double c = 0.0, const double h = 1.0) : _c(c), _h(h) {}

    double
    operator()(const Input& in) const {
        return std::exp(-_h * SQUARE(in - _c));
    }

private:
    double _c = 0.0;
    double _h = 1.0;
};

class PeriodicExponentialBasis {
public:
    using Input  = double;
    using Vector = std::vector<PeriodicExponentialBasis>;

    static Vector
    create_equispaced(const long n_basis) {
        // cs: set of desired base centers
        const double h = 200.0 / double(n_basis);
        Vector       basis(static_cast<std::size_t>(n_basis));
        for (std::size_t i = 0; i < basis.size(); ++i) {
            basis[i]._c = 2.0 * M_PI * double(i) / double(n_basis);
            basis[i]._h = h;
        }

        return basis;
    }

    PeriodicExponentialBasis(const double c = 0.0, const double h = 1.0) :
            _c(c), _h(h) {}

    double
    operator()(const Input& in) const {
        return std::exp(-_h * (1.0 - std::cos(in - _c)));
    }

    //  private:
    double _c = 0.0;
    double _h = 1.0;
};

template <int FunctionDimension, typename BaseFunction = ExponentialBasis>
class LearnableFunction {
public:
    using Input = BaseFunction::Input;

    using Basis         = std::vector<BaseFunction>;
    using WeightsMatrix = std::conditional_t<
            FunctionDimension == 1,
            Eigen::VectorXd,
            Eigen::Matrix<double, Eigen::Dynamic, FunctionDimension>>;
    using Output = std::conditional_t<
            FunctionDimension == 1,
            double,
            Eigen::Vector<double, FunctionDimension>>;

    LearnableFunction(std::vector<BaseFunction>&& basis) :
            _bs(std::move(basis)),
            _ws(WeightsMatrix::Zero(std::ssize(_bs), FunctionDimension)) {}

    template <typename Scale>
    void
    learn(const Eigen::MatrixXd& phi,
          const Eigen::MatrixXd& fdes,
          const Scale&           phi_weights)
        requires std::same_as<Scale, double>
    {
        assert(phi.cols() == static_cast<Eigen::Index>(n_basis()));
        assert(phi.rows() == fdes.rows());
        assert(fdes.cols() == FunctionDimension);
        _ws = fullpiv_qr_solve(phi_weights * phi, fdes);
    }

    void
    learn(const Eigen::MatrixXd& phi, const Eigen::MatrixXd& fdes) {
        return learn<double>(phi, fdes, 1.0);
    }

    template <typename Scale>
    void
    learn(const Eigen::MatrixXd& phi,
          const Eigen::MatrixXd& fdes,
          const Scale&           phi_weights) {
        assert(phi.cols() == static_cast<Eigen::Index>(n_basis()));
        assert(phi.rows() == fdes.rows());
        assert(fdes.cols() == FunctionDimension);
        for (long i = 0; i < FunctionDimension; ++i) {
            _ws.col(i) = fullpiv_qr_solve(phi_weights(i) * phi, fdes.col(i));
        }
    }

    Output
    operator()(const double s, const double scale = 1.0) const {
        const Eigen::VectorXd b = eval_basis(s);
        if constexpr (FunctionDimension == 1) return scale * b.transpose() * _ws;
        else return scale * (b.transpose() * _ws).transpose();
    };

    MDV_NODISCARD Eigen::VectorXd
                  eval_basis(const double s) const {
        Eigen::VectorXd basis(std::ssize(_bs));
        auto            basis_eval = [&s](const BaseFunction& b) { return b(s); };

        std::transform(_bs.begin(), _bs.end(), basis.begin(), basis_eval);
        basis /= basis.sum();  // Normalise
        return basis;
    }

    // clang-format off
    MDV_NODISCARD std::size_t            n_basis() const { return _bs.size(); }
    MDV_NODISCARD const WeightsMatrix& weights() const { return _ws; }
    MDV_NODISCARD WeightsMatrix& weights() { return _ws; }

    // clang-format on

private:
    Basis         _bs;
    WeightsMatrix _ws;
};

#undef SQUARE

}  // namespace mdv::dmp


#endif  // MDV_DMP_LEARNABLE_FUNCTION_HPP
