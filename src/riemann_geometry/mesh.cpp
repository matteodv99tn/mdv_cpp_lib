#include "mdv/riemann_geometry/mesh.hpp"

#include <Eigen/Dense>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/riemann_geometry/hypershere.hpp"

namespace mdv::riemann {
using Vec3 = Eigen::Vector3d;

Vec3
MeshManifold::logarithmic_map(const Point& p1, const Point& p2) const {
    return mdv::mesh::logarithmic_map(p1, p2).cartesian_vector();
}

MeshManifold::Point
MeshManifold::exponential_map(const Point& p, const TangentVector& v) const {
    return mdv::mesh::exponential_map(mesh::TangentVector::from_ambient_vector(p, v));
}

Vec3
MeshManifold::parallel_transport(
        const Point& q, const Point& p, const TangentVector& v
) const {
    assert(mdv::condition::are_orthogonal(
            mesh::normal(q.mesh(), mesh::FaceId{q.face().id()}), v
    ));
    return mdv::mesh::parallel_transport(
                   mesh::TangentVector::from_ambient_vector(q, v), p
    )
            .cartesian_vector();
}

Vec3
MeshManifold::covariant_derivative(const Point& p, const TangentVector& v) const {
    using S2          = mdv::riemann::S<2>;
    const Vec3 v_proj = S2::normal_projection(
            mesh::normal(p.mesh(), mesh::FaceId{p.face().id()}), v
    );
    return mesh::TangentVector::from_ambient_vector(p, v_proj).cartesian_vector();
}

MeshManifold::Point
MeshManifold::default_point() const {
    return {};
}

Vec3
MeshManifold::default_tangent_vector() const {
    return Vec3::Zero();
}

}  // namespace mdv::riemann
