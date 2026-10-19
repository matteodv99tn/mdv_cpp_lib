#include "mdv/mesh/half_edge.hpp"

#include <fmt/format.h>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh_element.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh {

HalfEdge::HalfEdge(const Vertex& v, const Face& f) :
        internal::IndexedMeshElement(v.mesh(), invalid_index) {
    assert(&v.mesh() == &f.mesh());

    using VertexIndex = internal::CgalImpl::CgalVertexIndex;
    using FaceIndex   = internal::CgalImpl::CgalFaceIndex;

    const VertexIndex v_id{v.id()};
    const FaceIndex   f_id{f.id()};
    const auto&       m = internal::get_mesh_impl(v.mesh());

    for (const auto he_id : CGAL::halfedges_around_face(CGAL::halfedge(f_id, m), m)) {
        if (source(he_id, m) == v_id) {
            set_id(he_id);
            break;
        }
    }

    if (id() == invalid_index) {
        const std::string msg = fmt::format(
                "Face #{} does not contain vertex #{}; cannot initialise proper "
                "halfedge",
                f_id.idx(),
                v_id.idx()
        );
        throw std::runtime_error(msg.c_str());
    };
}

Face
HalfEdge::face() const noexcept {
    const auto& m  = internal::get_mesh_impl(*this);
    const auto  id = CGAL::face(internal::to_halfedge_impl(*this), m);
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

HalfEdge
HalfEdge::next() const noexcept {
    const auto& m  = internal::get_mesh_impl(*this);
    const auto  id = internal::to_halfedge_impl(*this);
    return {mesh(), CGAL::next(id, m)};
}

HalfEdge
HalfEdge::prev() const noexcept {
    const auto& m  = internal::get_mesh_impl(*this);
    const auto  id = internal::to_halfedge_impl(*this);
    return {mesh(), CGAL::prev(id, m)};
}

HalfEdge
HalfEdge::twin() const noexcept {
    const auto& m  = internal::get_mesh_impl(*this);
    const auto  id = internal::to_halfedge_impl(*this);
    return {mesh(), CGAL::opposite(id, m)};
}

Vertex
HalfEdge::origin() const noexcept {
    const auto& m  = internal::get_mesh_impl(*this);
    const auto  id = internal::to_halfedge_impl(*this);
    return {mesh(), CGAL::source(id, m)};
}

std::string
HalfEdge::describe() const {
    return fmt::format(
            "HalfEdge on face #{} with origin vertex #{}", face().id(), origin().id()
    );
}

}  // namespace mdv::mesh
