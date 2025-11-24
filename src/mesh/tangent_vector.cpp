#include "mdv/mesh/tangent_vector.hpp"

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/helpers.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

// \cond DOXYGEN_IGNORE
using mdv::mesh::CartesianPoint;
using mdv::mesh::Mesh;
using mdv::mesh::TangentVector;

namespace {
mdv::Vec3d
normal_projection(const mdv::Vec3d& vec, const mdv::Vec3d& normal) {
    assert(mdv::condition::is_unit_norm(normal));
    return (mdv::Mat3d::Identity() - normal * normal.transpose()) * vec;
}

}  // namespace

// \endcond

//  _____                            _ __     __        _
// |_   _|_ _ _ __   __ _  ___ _ __ | |\ \   / /__  ___| |_ ___  _ __
//   | |/ _` | '_ \ / _` |/ _ \ '_ \| __\ \ / / _ \/ __| __/ _ \| '__|
//   | | (_| | | | | (_| |  __/ | | | |_ \ V /  __/ (__| || (_) | |
//   |_|\__,_|_| |_|\__, |\___|_| |_|\__| \_/ \___|\___|\__\___/|_|
//                  |___/
TangentVector::TangentVector(const Point& app_point, const Vec3d& v) :
        _pt(app_point), _vec(v) {
    using mdv::condition::are_orthogonal;

    // In case if the point is on a edge, you may switch the halfedge to ensure that the
    // vector is orthogonal to the face (which may be the opposite one)
    const auto* desc = _pt.get_as<Point::PointOnEdgeDescriptor>();
    const bool  shall_switch =
            (desc != nullptr) && !are_orthogonal(_pt.face().normal(), _vec);
    if (shall_switch) _pt = desc->display_in_opposite_halfedge();

    assert(mdv::condition::are_orthogonal(_pt.face().normal(), _vec));
}

TangentVector
TangentVector::from_tip_position(const Point& origin, const CartesianPoint& tip) {
    const Vec3d p0  = origin.position();
    const Vec3d vec = tip - p0;
    const Vec3d n   = origin.face().normal();
    return {origin, normal_projection(vec, n)};
}

TangentVector
TangentVector::unit_random(const Point& application_point) {
    const Vec3d n = application_point.face().normal();
    return {application_point, normal_projection(Vec3d::Random(), n).normalized()};
}

Eigen::Vector3d
TangentVector::tip() const noexcept {
    return application_point().position() + cartesian_vector();
}

mdv::Vec3d
TangentVector::cartesian_vector() const noexcept {
    return _vec;
}

void
TangentVector::scale(const double& factor) {
    _vec *= factor;
}

void
TangentVector::normalise() {
    _vec.normalize();
}

TangentVector
TangentVector::normalised() & {
    TangentVector res(*this);
    return {_pt, _vec.normalized()};
}

TangentVector
TangentVector::normalised() && {
    const double len = _vec.norm();
    scale(1.0 / len);
    return *this;
}
