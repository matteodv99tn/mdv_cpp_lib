#ifndef MDV_RIEMANN_MANIFOLD_HPP
#define MDV_RIEMANN_MANIFOLD_HPP

#include "mdv/riemann_geometry/concepts.hpp"

#define MDV_MANIFOLD_TYPENAMES_IMPORT(M)                                               \
    using Point         = typename M::Point;                                           \
    using TangentVector = typename M::TangentVector;

#endif  // MDV_RIEMANN_MANIFOLD_HPP
