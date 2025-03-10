#ifndef MDV_RIEMANN_MANIFOLD_HPP
#define MDV_RIEMANN_MANIFOLD_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>

#include "mdv/eigen_defines.hpp"
#include "mdv/riemann_geometry/hypershere.hpp"

#define MDV_MANIFOLD_TYPENAMES_IMPORT(M)                                               \
    using Point         = typename M::Point;                                           \
    using TangentVector = typename M::TangentVector;

namespace mdv::riemann {

template <typename M>
concept manifold = requires(M m, M::Point x, M::TangentVector v) {
    typename M::Point;
    typename M::TangentVector;

    { M::logarithmic_map(x, x) } -> std::same_as<typename M::TangentVector>;
    { M::exponential_map(x, v) } -> std::same_as<typename M::Point>;
    { M::covariant_derivative(x, v) } -> std::same_as<typename M::TangentVector>;
    { M::parallel_transport(x, x, v) } -> std::same_as<typename M::TangentVector>;

    { M::default_point() } -> std::same_as<typename M::Point>;
    { M::default_tangent_vector() } -> std::same_as<typename M::TangentVector>;
};


}  // namespace mdv::riemann


#endif  // MDV_RIEMANN_MANIFOLD_HPP
