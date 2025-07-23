#ifndef MDV_DMP_LEARNABLE_FUNCTION_HPP
#define MDV_DMP_LEARNABLE_FUNCTION_HPP

#include <cassert>
#include <Eigen/Dense>

#include "mdv/macros.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/utils/concepts.hpp"

#define SQUARE(x) ((x) * (x))

namespace mdv::dmp {

namespace internal {
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


    template <typename T>
    struct eigen_representation;

    template <typename T>
    using eigen_representation_t = typename eigen_representation<T>::type;

    template <concepts::eigen_vector T>
    struct eigen_representation<T> {
        using type = Eigen::Vector<double, type_elems_size_v<T>>;

        static type
        to_eigen(const T& data) {
            return data;
        }

        static T
        from_eigen(const type& data) {
            return data;
        }
    };

    template <>
    struct eigen_representation<double> {
        using type = Eigen::Vector<double, 1>;

        static type
        to_eigen(const double& data) {
            return type{data};
        }

        static double
        from_eigen(const type& data) {
            return data(0);
        }
    };

}  // namespace internal

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

template <typename LearnedType, typename BaseFunction = ExponentialBasis>
class LearnableFunction {
public:
    using BaseFun = BaseFunction;
    using Input   = BaseFun::Input;

    using basis_size_t = long;
    using weights_t    = Eigen::MatrixXd;

    using TangentVector = LearnedType;
    using EigenTanVec   = internal::eigen_representation_t<TangentVector>;
    static constexpr std::size_t tan_vec_dim = internal::type_elems_size_v<LearnedType>;

    LearnableFunction(std::vector<BaseFun>&& basis) : _basis(std::move(basis)) {}

    void
    learn(const Eigen::MatrixXd& phi, const Eigen::MatrixXd& fdes) {
        assert(phi.cols() == n_basis());
        assert(phi.rows() == fdes.rows());
        assert(fdes.cols() == tan_vec_dim);
        _ws = phi.fullPivHouseholderQr().solve(fdes);
    }

    TangentVector
    operator()(const double s, const double scale = 1.0) const {
        const Eigen::VectorXd b = eval_basis(s);
        return from_eigen(scale * b.transpose() * _ws);
    };

    Eigen::VectorXd
    eval_basis(const double s) const {
        Eigen::VectorXd basis(_basis.size());
        auto            basis_eval = [&s](const BaseFun& b) { return b(s); };

        std::transform(_basis.begin(), _basis.end(), basis.begin(), basis_eval);
        basis /= basis.sum();  // Normalise
        return basis;
    }

    // clang-format off
    MDV_NODISCARD std::size_t            n_basis() const { return _basis.size(); }
    MDV_NODISCARD const Eigen::MatrixXd& weights() const { return _ws; }

    // clang-format on

    static TangentVector
    from_eigen(const EigenTanVec& data) {
        return internal::eigen_representation<TangentVector>::from_eigen(data);
    }

    static EigenTanVec
    to_eigen(const TangentVector& data) {
        return internal::eigen_representation<TangentVector>::to_eigen(data);
    }

private:
    std::vector<BaseFun> _basis;
    weights_t            _ws;  // weights
};

#undef SQUARE

}  // namespace mdv::dmp


#endif  // MDV_DMP_LEARNABLE_FUNCTION_HPP
