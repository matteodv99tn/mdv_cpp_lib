#ifndef MDV_DMP_CONCEPTS_HPP
#define MDV_DMP_CONCEPTS_HPP

#include <type_traits>

#include "mdv/containers/demonstration.hpp"
#include "mdv/riemann_geometry/manifold.hpp"

namespace mdv {

template <typename TS, typename M>
concept transformation_system = requires(TS ts) {
    requires riemann::manifold<M>;
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


#endif  // MDV_DMP_CONCEPTS_HPP
