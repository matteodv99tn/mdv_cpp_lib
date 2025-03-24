#include "mdv/riemann_geometry/mesh.hpp"

#include <Eigen/Dense>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"

using MeshManifold = mdv::riemann::MeshManifold;

using Point      = mdv::mesh::Mesh::Point;
using Vec3       = Eigen::Vector3d;
using MeshTanVec = mdv::mesh::TangentVector;

Vec3
MeshManifold::logarithmic_map(const Point& p1, const Point& p2) {
    return mdv::mesh::logarithmic_map(p1, p2).cartesian_vector();
}

Point
MeshManifold::exponential_map(const Point& p, const TangentVector& v) {
    return mdv::mesh::exponential_map(MeshTanVec(p, v));
}

Vec3
MeshManifold::parallel_transport(
        const Point& q, const Point& p, const TangentVector& v
) {
    return mdv::mesh::parallel_transport(MeshTanVec(q, v), p).cartesian_vector();
}

Vec3
MeshManifold::covariant_derivative(const Point& p, const TangentVector& v) {
    return MeshTanVec(p, v).cartesian_vector();
}

Point
MeshManifold::default_point() {
    return {};
}

Vec3
MeshManifold::default_tangent_vector() {
    return Vec3::Zero();
}
