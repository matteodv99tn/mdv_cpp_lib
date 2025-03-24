#ifndef MDV_MESH_MANIFOLD_HPP
#define MDV_MESH_MANIFOLD_HPP

#include <Eigen/Dense>

#include "mdv/mesh/mesh.hpp"

namespace mdv::riemann {

class MeshManifold {
public:
    using Point         = mdv::mesh::Mesh::Point;
    using TangentVector = Eigen::Vector3d;

    static TangentVector logarithmic_map(const Point& q1, const Point& q2);

    static Point exponential_map(const Point& q, const TangentVector& v);

    static TangentVector parallel_transport(
            const Point& q, const Point& p, const TangentVector& v
    );

    static TangentVector covariant_derivative(const Point& q, const TangentVector& v);

    static Point default_point();

    static TangentVector default_tangent_vector();
};
}  // namespace mdv::riemann

#endif  // MDV_MESH_MANIFOLD_HPP
