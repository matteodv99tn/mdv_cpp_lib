#ifndef MDV_MESH_CGAL_DATA_HPP
#define MDV_MESH_CGAL_DATA_HPP

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path_traits.h>
#include <filesystem>
#include <gsl/pointers>
#include <optional>

#include "mdv/mesh/face.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/mesh/vertex.hpp"


#if MDV_CGAL_VERSION == 5
#include <CGAL/AABB_traits.h>
#elif MDV_CGAL_VERSION == 6
#include <CGAL/AABB_traits_3.h>
#endif

#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/logging.hpp"

namespace mdv::mesh::internal {

class CgalGeodesicConstructor;

/**
 * @brief Implementation class for mesh data using CGAL.
 *
 * This class encapsulates the necessary data and functionality to work with
 * meshes using the Computational Geometry Algorithms Library (CGAL). It
 * provides methods for loading, processing, and querying mesh data.
 */
class CgalImpl {
public:
    // CGAL typedefs - general
    using Kernel    = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point3    = Kernel::Point_3;
    using Mesh      = CGAL::Surface_mesh<Point3>;
    using Transform = Kernel::Aff_transformation_3;
    using Vec3      = Kernel::Vector_3;

    // CGAL typedefs - shortest path
    using ShortestPathTraits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Mesh>;
    using ShortestPath       = CGAL::Surface_mesh_shortest_path<ShortestPathTraits>;

    // CGAL typedefs - AABB tree
    using AabbPrimitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
#if MDV_CGAL_VERSION == 5
    using AabbTraits = CGAL::AABB_traits<Kernel, AabbPrimitive>;
#elif MDV_CGAL_VERSION == 6
    using AabbTraits = CGAL::AABB_traits_3<Kernel, AabbPrimitive>;
#endif
    using AabbTree = CGAL::AABB_tree<AabbTraits>;

    // CGAL typedefs - variable access
    using CgalVertexIndex   = Mesh::Vertex_index;
    using CgalFaceIndex     = Mesh::Face_index;
    using CgalHalfEdgeIndex = Mesh::Halfedge_index;
    using FaceLocation      = ShortestPath::Face_location;

    // CGAL typedefs - descriptors
    using VertexDescriptor = boost::graph_traits<Mesh>::vertex_descriptor;

    // Factory functions
    static gsl::owner<CgalImpl*> from_file(
            const std::filesystem::path& file_path, Logger::SharedPtr&& logger
    );

    CgalImpl(const Mesh&& mesh, Logger::SharedPtr&& logger);
    CgalImpl(const CgalImpl&)            = delete;
    CgalImpl& operator=(const CgalImpl&) = delete;
    CgalImpl(CgalImpl&&) noexcept;
    CgalImpl& operator=(CgalImpl&&) noexcept = delete;
    ~CgalImpl();

    std::unique_ptr<ShortestPath>                _shortest_path;
    mutable Logger::SharedPtr                    _logger;
    Mesh                                         _mesh;
    AabbTree                                     _aabb_tree;
    mutable gsl::owner<mdv::mesh::Point*>        _current_shortpath_source = nullptr;
    mutable gsl::owner<CgalGeodesicConstructor*> _geodesic_constructor     = nullptr;

    void build_vertex_normals_map() noexcept;

    MDV_NODISCARD std::vector<Eigen::Vector3d> yield_vertices() const;

    MDV_NODISCARD std::vector<IndexTriplet> yield_faces() const;

    Logger&
    logger() const {
        assert(_logger);
        return *_logger;
    };
};

CgalImpl::FaceLocation location_from_mesh_point(const ::mdv::mesh::Point& pt) noexcept;

//   ____                              _
//  / ___|___  _ ____   _____ _ __ ___(_) ___  _ __
// | |   / _ \| '_ \ \ / / _ \ '__/ __| |/ _ \| '_ \
// | |__| (_) | | | \ V /  __/ |  \__ \ | (_) | | | |
//  \____\___/|_| |_|\_/ \___|_|  |___/_|\___/|_| |_|
//
//  _          _
// | |__   ___| |_ __   ___ _ __ ___
// | '_ \ / _ \ | '_ \ / _ \ '__/ __|
// | | | |  __/ | |_) |  __/ |  \__ \
// |_| |_|\___|_| .__/ \___|_|  |___/
//              |_|
Eigen::Vector3d convert(const CgalImpl::Vec3& x);
Eigen::Vector3d convert(const CgalImpl::Point3& x);

MDV_INLINE CgalImpl::Kernel::Point_3
           point3_from_eigen(const Vec3d& vec) {
    return {vec(0), vec(1), vec(2)};
};

MDV_INLINE CgalImpl::Kernel::Direction_3
           direction3_from_eigen(const Vec3d& vec) {
    return {vec(0), vec(1), vec(2)};
};

MDV_INLINE CgalImpl::Kernel::Vector_3
           vector3_from_eigen(const Vec3d& vec) {
    return {vec(0), vec(1), vec(2)};
};

MDV_INLINE const CgalImpl::Mesh&
                 get_mesh_impl(const MeshElement& elem) {
    return elem.mesh().cgal()._mesh;
};

MDV_INLINE const CgalImpl::Mesh&
                 get_mesh_impl(const Mesh& mesh) {
    return mesh.cgal()._mesh;
};

MDV_INLINE CgalImpl::Point3
           to_vertex_impl(const Vertex& v) {
    CgalImpl::CgalVertexIndex id(v.id());
    return v.mesh().cgal()._mesh.point(id);
}

MDV_INLINE CgalImpl::CgalFaceIndex
           to_face_impl(const Face& f) {
    return CgalImpl::CgalFaceIndex{f.id()};
}

MDV_INLINE CgalImpl::CgalHalfEdgeIndex
           to_halfedge_impl(const HalfEdge& he) {
    return CgalImpl::CgalHalfEdgeIndex{he.id()};
}

MDV_INLINE CgalImpl::Kernel::Point_3
           point3_from_point(const Point& pt) {
    return point3_from_eigen(pt.position());
}

MDV_INLINE CgalImpl::Kernel::Ray_3
           ray3_from_tangent_vector(const TangentVector& tv) {
    return {point3_from_point(tv.application_point()),
                       direction3_from_eigen(tv.cartesian_vector())};
}

MDV_INLINE CgalImpl::Kernel::Triangle_3
           triangle3_from_face(const Face& f) {
    const auto& m   = get_mesh_impl(f);
    const auto  he0 = m.halfedge(to_face_impl(f));
    const auto  he1 = next(he0, m);
    const auto  he2 = next(he1, m);
    const auto& p0  = m.point(source(he0, m));
    const auto& p1  = m.point(source(he1, m));
    const auto& p2  = m.point(source(he2, m));
    return {p0, p1, p2};
}

/**
 * If existing, returns the halfedge of the source face which is shared with the target
 * face.
 */
MDV_INLINE std::optional<CgalImpl::CgalHalfEdgeIndex>
           shared_halfedge(
                   const CgalImpl::CgalFaceIndex& source_face,
                   const CgalImpl::CgalFaceIndex& target_face,
                   const CgalImpl::Mesh&          mesh
           ) {
    const auto source_he = halfedge(source_face, mesh);
    for (const auto he : CGAL::halfedges_around_face(source_he, mesh))
        if (face(opposite(he, mesh), mesh) == target_face) return he;

    return std::nullopt;
}

Eigen::Quaterniond relative_face_rotation(
        const CgalImpl::CgalHalfEdgeIndex& he, const CgalImpl::Mesh& mesh
);

std::optional<CgalImpl::Kernel::Point_3> edge_ray_intersection(
        const CgalImpl::Kernel::Segment_3& edge, const CgalImpl::Kernel::Ray_3& ray
);

double total_curvature_rad(
        const CgalImpl::Mesh& m, const CgalImpl::VertexDescriptor& v
);

double total_curvature_deg(
        const CgalImpl::Mesh& m, const CgalImpl::VertexDescriptor& v
);

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_CGAL_DATA_HPP
