#include "mdv/mesh/half_edge.hpp"

#include <fmt/format.h>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/utils/conditions.hpp"

// \cond DOXYGEN_IGNORE
using mdv::mesh::Face;
using mdv::mesh::HalfEdge;
using mdv::mesh::Vertex;

// \endcond


Face
HalfEdge::face() const noexcept {
    const auto m  = internal::get_mesh_impl(*this);
    const auto id = CGAL::face(internal::to_halfedge_impl(*this), m);
    return {mesh(), id};
}

Face
HalfEdge::opposite_face() const noexcept {
    const auto m           = internal::get_mesh_impl(*this);
    const auto he_id       = internal::to_halfedge_impl(*this);
    const auto opposite_id = CGAL::opposite(he_id, m);
    const auto id          = CGAL::face(opposite_id, m);
    return {mesh(), id};
}

Eigen::Vector3d
HalfEdge::inbound_direction() const {
    using Vec3   = Eigen::Vector3d;
    using Mat3   = Eigen::Matrix3d;
    const Vec3 e = normalised_direction();         // current edge
    const Vec3 v = next().normalised_direction();  // vector w.r.t. which will be
                                                   // rendered orthoghonal to e

    const Mat3 proj = (Mat3::Identity() - e * e.transpose());
    const Vec3 res  = (proj * v).normalized();

    assert(mdv::condition::are_orthogonal(e, res));
    assert(mdv::condition::are_orthogonal(face().normal(), res));

    return res;
}

Eigen::Quaterniond
HalfEdge::aligning_rotation() const {
    using Vec3 = Eigen::Vector3d;
    using Quat = Eigen::Quaterniond;

    const Vec3 e       = normalised_direction();
    const Vec3 x_this  = inbound_direction();
    const Vec3 x_other = twin().inbound_direction();

    assert(mdv::condition::are_orthogonal(x_this, e));
    assert(mdv::condition::are_orthogonal(x_other, e));

    const Quat res = Quat::FromTwoVectors(-x_this, x_other);
    assert(mdv::condition::are_equal(res * (-x_this), x_other));
    assert(mdv::condition::are_equal(res * e, e));
    return res;
}

HalfEdge
HalfEdge::next() const noexcept {
    const auto m  = internal::get_mesh_impl(*this);
    const auto id = internal::to_halfedge_impl(*this);
    return {mesh(), CGAL::next(id, m)};
}

HalfEdge
HalfEdge::prev() const noexcept {
    const auto m  = internal::get_mesh_impl(*this);
    const auto id = internal::to_halfedge_impl(*this);
    return {mesh(), CGAL::prev(id, m)};
}

HalfEdge
HalfEdge::twin() const noexcept {
    const auto m  = internal::get_mesh_impl(*this);
    const auto id = internal::to_halfedge_impl(*this);
    return {mesh(), CGAL::opposite(id, m)};
}

Vertex
HalfEdge::origin() const noexcept {
    const auto m  = internal::get_mesh_impl(*this);
    const auto id = internal::to_halfedge_impl(*this);
    return {mesh(), CGAL::source(id, m)};
}

std::string
HalfEdge::describe() const {
    return fmt::format(
            "HalfEdge on face #{} with origin vertex #{}", face().id(), origin().id()
    );
}

bool
HalfEdge::is_opposite_of(const HalfEdge& other) const noexcept {
    const auto m        = internal::get_mesh_impl(*this);
    const auto this_id  = internal::to_halfedge_impl(*this);
    const auto other_id = internal::to_halfedge_impl(other);
    return this_id == CGAL::opposite(other_id, m);
}
