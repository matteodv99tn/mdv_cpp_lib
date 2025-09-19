#ifndef MDV_MESH_MANIFOLD_HPP
#define MDV_MESH_MANIFOLD_HPP

#include <Eigen/Dense>

#include "mdv/mesh/mesh.hpp"

namespace mdv::riemann {

class MeshManifold {
public:
    using Point         = mdv::mesh::Point;
    using TangentVector = Eigen::Vector3d;

    TangentVector logarithmic_map(const Point& q1, const Point& q2) const;

    Point exponential_map(const Point& q, const TangentVector& v) const;

    TangentVector parallel_transport(
            const Point& q, const Point& p, const TangentVector& v
    ) const;

    TangentVector covariant_derivative(const Point& q, const TangentVector& v) const;

    Point default_point() const;

    TangentVector default_tangent_vector() const;
};
}  // namespace mdv::riemann

#endif  // MDV_MESH_MANIFOLD_HPP
