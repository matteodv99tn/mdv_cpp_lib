#ifndef MDV_DMP_CONCEPTS_HPP
#define MDV_DMP_CONCEPTS_HPP

#include <concepts>
#include <type_traits>

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/fwd.hpp"
#include "mdv/riemann_geometry/concepts.hpp"

namespace mdv {

template <typename TS, typename M>
concept transformation_system = requires(TS ts) {
    requires riemann::concepts::manifold<M>;
    typename TS::Manifold;
    std::is_same_v<typename TS::Manifold, M>;
    std::is_same_v<typename TS::Point, typename M::Point>;
    std::is_same_v<typename TS::TangentVector, typename M::TangentVector>;

    // Check existance of default types
    typename TS::MinimumSample;      // Minimum sized sample container for describing
                                     // the current state of the transformation system
    typename TS::MinimumGoalSample;  // Minimum sized sample container for describing
                                     // the goal state of the transformation system
    requires manifold_sample<typename TS::MinimumSample, 0, M>;
    requires manifold_sample<typename TS::MinimumGoalSample, 0, M>;
};


}  // namespace mdv

namespace mdv::concepts {

using namespace ::mdv::riemann::concepts;

template <typename Embedding, typename Manifold>
concept embedding = requires(Embedding e, Manifold m) {
    requires manifold<Manifold>;
    typename Embedding::Input;
    typename Embedding::Output;

    requires std::constructible_from<const Embedding, Manifold*>;
    requires euclidean_type<typename Embedding::Output>;
};

template <typename Manifold>
concept default_embeddable = requires {
    requires manifold<Manifold>;
    requires euclidean_type<typename Manifold::TangentVector>;
};

template <typename Manifold>
concept trivially_embeddable = requires {
    requires manifold<Manifold>;
    typename TrivialTypeEmbedding<typename Manifold::TangentVector>::Input;
    typename TrivialTypeEmbedding<typename Manifold::TangentVector>::Output;
    requires std::is_default_constructible_v<
            TrivialTypeEmbedding<typename Manifold::TangentVector>>;
};


}  // namespace mdv::concepts


#endif  // MDV_DMP_CONCEPTS_HPP
