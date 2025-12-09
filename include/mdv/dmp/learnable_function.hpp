#ifndef MDV_DMP_LEARNABLE_FUNCTION_HPP
#define MDV_DMP_LEARNABLE_FUNCTION_HPP

#include <cassert>
#include <cmath>
#include <Eigen/Dense>
#include <type_traits>
#include <vector>

#include "mdv/macros.hpp"

#define SQUARE(x) ((x) * (x))

namespace mdv::dmp {

class ExponentialBasis {
public:
    using Input  = double;
    using Vector = std::vector<ExponentialBasis>;

    static Vector
    create_from_centers(const Eigen::VectorXd& cs) {
        // cs: set of desired base centers
        Vector basis(cs.size());
        for (long i = 0; i < cs.size(); ++i) {
            basis[i]._c = cs(i);
            if (i < cs.size() - 1) basis[i]._h = 1 / SQUARE(cs(i + 1) - cs(i));
            else basis[i]._h = basis[i - 1]._h;
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
        Vector       basis(n_basis);
        for (long i = 0; i < n_basis; ++i) {
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
            _ws(WeightsMatrix::Zero(_bs.size(), FunctionDimension)) {}

    template <typename Scale>
    void
    learn(const Eigen::MatrixXd& phi,
          const Eigen::MatrixXd& fdes,
          const Scale&           phi_weights)
        requires std::same_as<Scale, double>
    {
        assert(phi.cols() == n_basis());
        assert(phi.rows() == fdes.rows());
        assert(fdes.cols() == FunctionDimension);
        _ws = (phi_weights * phi).fullPivHouseholderQr().solve(fdes);
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
        assert(phi.cols() == n_basis());
        assert(phi.rows() == fdes.rows());
        assert(fdes.cols() == FunctionDimension);
        for (long i = 0; i < FunctionDimension; ++i) {
            _ws.col(i) =
                    (phi_weights(i) * phi).fullPivHouseholderQr().solve(fdes.col(i));
        }
    }

    Output
    operator()(const double s, const double scale = 1.0) const {
        const Eigen::VectorXd b = eval_basis(s);
        if constexpr (FunctionDimension == 1) return scale * b.transpose() * _ws;
        else return scale * (b.transpose() * _ws).transpose();
    };

    Eigen::VectorXd
    eval_basis(const double s) const {
        Eigen::VectorXd basis(_bs.size());
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
