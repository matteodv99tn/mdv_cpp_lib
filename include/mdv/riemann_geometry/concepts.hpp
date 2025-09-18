#ifndef MDV_RIEMANN_CONCEPTS_HPP
#define MDV_RIEMANN_CONCEPTS_HPP

#include <concepts>

#include "mdv/utils/concepts.hpp"

namespace mdv::riemann::concepts {

template <typename M>
concept manifold = requires(M m, M::Point x, M::TangentVector v) {
    typename M::Point;
    typename M::TangentVector;

    { m.logarithmic_map(x, x) } -> std::same_as<typename M::TangentVector>;
    { m.exponential_map(x, v) } -> std::same_as<typename M::Point>;
    { m.covariant_derivative(x, v) } -> std::same_as<typename M::TangentVector>;
    { m.parallel_transport(x, x, v) } -> std::same_as<typename M::TangentVector>;
    { m.default_point() } -> std::same_as<typename M::Point>;
    { m.default_tangent_vector() } -> std::same_as<typename M::TangentVector>;
};

template <typename T>
concept euclidean_type = requires {
    requires ::mdv::concepts::eigen_vector_like<T> || std::same_as<T, double>;
};

template <typename M>
concept euclidean_space = requires {
    requires manifold<M>;
    requires std::same_as<typename M::Point, typename M::TangentVector>;
    requires euclidean_type<typename M::Point>;
};

}  // namespace mdv::riemann::concepts


#endif  // MDV_RIEMANN_CONCEPTS_HPP
