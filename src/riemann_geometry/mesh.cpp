#include "mdv/riemann_geometry/mesh.hpp"

#include <Eigen/Dense>
#include <stdexcept>
#include <utility>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/riemann_geometry/hypershere.hpp"

namespace mdv::riemann {
using Vec3 = Eigen::Vector3d;

MeshManifold::MeshManifold(std::shared_ptr<const mesh::Mesh> mesh) :
        _mesh(std::move(mesh)) {
    if (!_mesh) throw std::invalid_argument("MeshManifold requires a mesh owner");
}

const mesh::Mesh&
MeshManifold::mesh() const {
    return *_mesh;
}

Vec3
MeshManifold::logarithmic_map(const Point& p1, const Point& p2) const {
    return mesh::logarithmic_map(mesh(), p1, p2).cartesian_vector();
}

MeshManifold::Point
MeshManifold::exponential_map(const Point& p, const TangentVector& v) const {
    return mesh::exponential_map(
            mesh(), mesh::TangentVector::from_ambient_vector(mesh(), p, v)
    );
}

Vec3
MeshManifold::parallel_transport(
        const Point& q, const Point& p, const TangentVector& v
) const {
    assert(mdv::condition::are_orthogonal(
            mesh::normal(mesh(), mesh::face(mesh(), q)), v
    ));
    return mdv::mesh::parallel_transport(
                   mesh(), mesh::TangentVector::from_ambient_vector(mesh(), q, v), p
    )
            .cartesian_vector();
}

Vec3
MeshManifold::covariant_derivative(const Point& p, const TangentVector& v) const {
    using S2 = S<2>;
    const Vec3 v_proj =
            S2::normal_projection(mesh::normal(mesh(), mesh::face(mesh(), p)), v);
    return mesh::TangentVector::from_ambient_vector(mesh(), p, v_proj)
            .cartesian_vector();
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
