#ifndef MDV_RIEMANN_CONCEPTS_HPP
#define MDV_RIEMANN_CONCEPTS_HPP

#include <concepts>
#include <type_traits>

#include "mdv/riemann_geometry/fwd.hpp"
#include "mdv/utils/concepts.hpp"

namespace mdv::concepts {

template <typename M>
concept manifold = requires(const M m, M::Point x, M::TangentVector v) {
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

namespace internal {

    template <typename T, typename = void>
    struct HasTrivialTypeEmbedding : std::false_type {};

    template <typename T>
    struct HasTrivialTypeEmbedding<
            T,
            std::void_t<decltype(sizeof(riemann::TrivialTypeEmbedding<T>))>>
            : std::true_type {};

    template <typename T>
    concept trivial_type_requirements = requires {
        typename riemann::TrivialTypeEmbedding<T>::Input;
        typename riemann::TrivialTypeEmbedding<T>::Output;

        std::is_default_constructible_v<riemann::TrivialTypeEmbedding<T>>;
    };


}  // namespace internal

template <typename T>
concept trivially_embeddable_type = internal::HasTrivialTypeEmbedding<T>::value
                                    && internal::trivial_type_requirements<T>;
}  // namespace mdv::concepts


#endif  // MDV_RIEMANN_CONCEPTS_HPP
