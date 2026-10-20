#include "mdv/mesh/half_edge.hpp"

#include <fmt/format.h>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh {

HalfEdgeId
halfedge(const Mesh& mesh, VertexId v, FaceId f) {
    const auto  v_id = internal::to_cgal(v);
    const auto  f_id = internal::to_cgal(f);
    const auto& m    = internal::cgal_mesh(mesh);
    for (const auto he_id : CGAL::halfedges_around_face(CGAL::halfedge(f_id, m), m)) {
        if (source(he_id, m) == v_id) return internal::from_cgal(he_id);
    }
    throw std::runtime_error(
            fmt::format(
                    "Face #{} does not contain vertex #{}; cannot initialise proper "
                    "halfedge",
                    f_id.idx(),
                    v_id.idx()
            )
    );
}

FaceId
face(const Mesh& mesh, HalfEdgeId h) {
    return internal::from_cgal(
            CGAL::face(internal::to_cgal(h), internal::cgal_mesh(mesh))
    );
}

Vec3d
direction(const Mesh& mesh, HalfEdgeId h) {
    const auto& m  = internal::cgal_mesh(mesh);
    const auto  id = internal::to_cgal(h);
    return position(mesh, internal::from_cgal(CGAL::source(CGAL::next(id, m), m)))
           - position(mesh, internal::from_cgal(CGAL::source(id, m)));
}

Vec3d
inbound_direction(const Mesh& mesh, HalfEdgeId h) {
    const auto& m = internal::cgal_mesh(mesh);
    const Vec3d e = direction(mesh, h).normalized();
    const Vec3d v =
            direction(mesh, internal::from_cgal(CGAL::next(internal::to_cgal(h), m)))
                    .normalized();
    const Mat3d proj = (Mat3d::Identity() - e * e.transpose());
    const Vec3d res  = (proj * v).normalized();
    assert(condition::are_orthogonal(e, res));
    assert(condition::are_orthogonal(normal(mesh, face(mesh, h)), res));
    return res;
}

HalfEdgeId
prev(const Mesh& mesh, HalfEdgeId h) {
    return internal::from_cgal(
            CGAL::prev(internal::to_cgal(h), internal::cgal_mesh(mesh))
    );
}

HalfEdgeId
twin(const Mesh& mesh, HalfEdgeId h) {
    return internal::from_cgal(
            CGAL::opposite(internal::to_cgal(h), internal::cgal_mesh(mesh))
    );
}

std::string
HalfEdge::describe() const {
    const auto& m = internal::cgal_mesh(mesh());
    const auto  h = internal::to_cgal(HalfEdgeId{id()});
    return fmt::format(
            "HalfEdge on face #{} with origin vertex #{}",
            CGAL::face(h, m).idx(),
            CGAL::source(h, m).idx()
    );
}

}  // namespace mdv::mesh
