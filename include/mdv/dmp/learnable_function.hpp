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

template <riemann::manifold M>
class LearnableFunction {
public:
    using basis_size_t = long;
    using weights_t    = Eigen::MatrixXd;

    using TangentVector = M::TangentVector;
    using EigenTanVec   = internal::eigen_representation_t<TangentVector>;
    static constexpr std::size_t tan_vec_dim = internal::type_elems_size_v<EigenTanVec>;

    LearnableFunction(basis_size_t num_basis) :
            _n_basis(num_basis),
            _basis_c(Eigen::VectorXd::Zero(num_basis)),
            _basis_h(Eigen::VectorXd::Zero(num_basis)),
            _ws(Eigen::MatrixXd::Zero(num_basis, tan_vec_dim)) {}

    void
    learn(const Eigen::MatrixXd& phi, const Eigen::MatrixXd& fdes) {
        assert(phi.cols() == _n_basis);
        assert(phi.rows() == fdes.rows());
        assert(fdes.cols() == tan_vec_dim);
        _ws = phi.fullPivHouseholderQr().solve(fdes);
    }

    void
    assign_centers(Eigen::VectorXd&& centers) {
        assert(centers.size() == _n_basis);
        _basis_c = std::move(centers);
    }

    void
    assign_widths(Eigen::VectorXd&& widths) {
        assert(widths.size() == _n_basis);
        _basis_h = std::move(widths);
    }

    TangentVector
    eval(const double s, const double scale = 1.0) const {
        const Eigen::VectorXd b = eval_basis(s);
        return from_eigen(scale * b.transpose() * _ws);
    };

    Eigen::VectorXd
    eval_basis(const double s) const {
        Eigen::VectorXd basis(_n_basis);
        for (auto i = 0; i < _n_basis; ++i)
            basis(i) = std::exp(-_basis_h[i] * SQUARE(s - _basis_c[i]));
        basis /= basis.sum();  // Normalise
        return basis;
    }

    // clang-format off
    MDV_NODISCARD std::size_t            n_basis() const { return _n_basis; }
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
    basis_size_t    _n_basis;
    Eigen::VectorXd _basis_c;  // Centers of the basis
    Eigen::VectorXd _basis_h;  // h parameters of the basis
    weights_t       _ws;       // weights
};

#undef SQUARE

}  // namespace mdv::dmp


#endif  // MDV_DMP_LEARNABLE_FUNCTION_HPP
