#ifndef MDV_DMP_LEARNABLE_FUNCTION_HPP
#define MDV_DMP_LEARNABLE_FUNCTION_HPP

#include <cassert>
#include <Eigen/Dense>
#include <type_traits>

#include "mdv/macros.hpp"

#define SQUARE(x) ((x) * (x))

namespace mdv::dmp {

class ExponentialBasis {
public:
    using Input = double;

    ExponentialBasis(const double c, const double h) : _c(c), _h(h) {}

    double
    operator()(const Input& in) const {
        return std::exp(-_h * SQUARE(in - _c));
    }

private:
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

    LearnableFunction(std::vector<BaseFunction>&& basis) : _bs(std::move(basis)) {}

    void
    learn(const Eigen::MatrixXd& phi, const Eigen::MatrixXd& fdes) {
        assert(phi.cols() == n_basis());
        assert(phi.rows() == fdes.rows());
        assert(fdes.cols() == FunctionDimension);
        _ws = phi.fullPivHouseholderQr().solve(fdes);
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

    // clang-format on

private:
    Basis         _bs;
    WeightsMatrix _ws;
};

#undef SQUARE

}  // namespace mdv::dmp


#endif  // MDV_DMP_LEARNABLE_FUNCTION_HPP
