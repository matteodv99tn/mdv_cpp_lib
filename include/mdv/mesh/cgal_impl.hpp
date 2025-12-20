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

#include "mdv/macros.hpp"
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
 * @brief CGAL-backed implementation of mesh storage and algorithms.
 *
 * This internal class owns CGAL data structures for the mesh, shortest-path
 * computations, and spatial queries. It is hidden behind the Mesh PIMPL to
 * reduce compilation time while exposing the geometric capabilities required
 * for policy learning on surfaces.
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

    /**
     * @brief Loads a CGAL mesh from file and builds its auxiliary structures.
     *
     * @param file_path Path to mesh file.
     * @param logger Logger instance to use.
     * @return Owned CGAL implementation pointer.
     */
    static gsl::owner<CgalImpl*> from_file(
            const std::filesystem::path& file_path, Logger::SharedPtr&& logger
    );

    /**
     * @brief Constructs from a CGAL mesh instance.
     *
     * @param mesh CGAL mesh.
     * @param logger Logger instance.
     */
    CgalImpl(Mesh&& mesh, Logger::SharedPtr&& logger);

    /**
     * @brief Non-copyable.
     */
    CgalImpl(const CgalImpl&) = delete;

    /**
     * @brief Non-copyable.
     */
    CgalImpl& operator=(const CgalImpl&) = delete;

    /**
     * @brief Move constructor.
     *
     * @param other Instance to move from.
     */
    CgalImpl(CgalImpl&&) noexcept;

    /**
     * @brief Non-movable assignment.
     */
    CgalImpl& operator=(CgalImpl&&) noexcept = delete;

    /**
     * @brief Destructor.
     */
    ~CgalImpl();

    std::unique_ptr<ShortestPath>                _shortest_path;
    mutable Logger::SharedPtr                    _logger;
    Mesh                                         _mesh;
    AabbTree                                     _aabb_tree;
    mutable gsl::owner<mdv::mesh::Point*>        _current_shortpath_source = nullptr;
    mutable gsl::owner<CgalGeodesicConstructor*> _geodesic_constructor     = nullptr;

    void scale(double factor);

    void transform(const Eigen::Affine3d& transformation);

    /**
     * @brief Builds or retrieves the per-vertex normal property map.
     */
    void build_vertex_normals_map() noexcept;

    /**
     * @brief Returns all mesh vertices as Eigen vectors.
     *
     * @return Vertex positions.
     */
    MDV_NODISCARD std::vector<Eigen::Vector3d> yield_vertices() const;

    /**
     * @brief Returns all mesh faces as index triplets.
     *
     * @return Face index triplets.
     */
    MDV_NODISCARD std::vector<IndexTriplet> yield_faces() const;

    /**
     * @brief Access logger.
     *
     * @return Logger reference.
     */
    Logger&
    logger() const {
        assert(_logger);
        return *_logger;
    };
};

/**
 * @brief Converts a mesh point to a CGAL face location (face + barycentric).
 *
 * @param pt Mesh point.
 * @return CGAL face location.
 */
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
/**
 * @brief Converts CGAL vectors to Eigen.
 *
 * @param x CGAL vector.
 * @return Eigen vector.
 */
MDV_INLINE Eigen::Vector3d
           convert(const CgalImpl::Vec3& x) {
    return {x.x(), x.y(), x.z()};
}

/**
 * @brief Converts CGAL points to Eigen.
 *
 * @param x CGAL point.
 * @return Eigen vector.
 */
MDV_INLINE Eigen::Vector3d
           convert(const CgalImpl::Point3& x) {
    return {x.x(), x.y(), x.z()};
}

/**
 * @brief Converts Eigen 3D vector to CGAL point.
 *
 * @param vec Eigen vector.
 * @return CGAL point.
 */
MDV_INLINE CgalImpl::Kernel::Point_3
           point3_from_eigen(const Vec3d& vec) {
    return {vec(0), vec(1), vec(2)};
};

/**
 * @brief Converts Eigen 3D vector to CGAL direction.
 *
 * @param vec Eigen vector.
 * @return CGAL direction.
 */
MDV_INLINE CgalImpl::Kernel::Direction_3
           direction3_from_eigen(const Vec3d& vec) {
    return {vec(0), vec(1), vec(2)};
};

/**
 * @brief Converts Eigen 3D vector to CGAL vector.
 *
 * @param vec Eigen vector.
 * @return CGAL vector.
 */
MDV_INLINE CgalImpl::Kernel::Vector_3
           vector3_from_eigen(const Vec3d& vec) {
    return {vec(0), vec(1), vec(2)};
};

/**
 * @brief Retrieves CGAL mesh from a mesh element.
 *
 * @param elem Mesh element.
 * @return CGAL mesh reference.
 */
MDV_INLINE const CgalImpl::Mesh&
                 get_mesh_impl(const MeshElement& elem) {
    return elem.mesh().cgal()._mesh;
};

/**
 * @brief Retrieves CGAL mesh from a mesh instance.
 *
 * @param mesh Mesh instance.
 * @return CGAL mesh reference.
 */
MDV_INLINE const CgalImpl::Mesh&
                 get_mesh_impl(const Mesh& mesh) {
    return mesh.cgal()._mesh;
};

/**
 * @brief Converts a mesh vertex to CGAL point.
 *
 * @param v Mesh vertex.
 * @return CGAL point.
 */
MDV_INLINE CgalImpl::Point3
           to_vertex_impl(const Vertex& v) {
    CgalImpl::CgalVertexIndex id(v.id());
    return v.mesh().cgal()._mesh.point(id);
}

/**
 * @brief Converts a mesh face to CGAL face index.
 *
 * @param f Mesh face.
 * @return CGAL face index.
 */
MDV_INLINE CgalImpl::CgalFaceIndex
           to_face_impl(const Face& f) {
    return CgalImpl::CgalFaceIndex{f.id()};
}

/**
 * @brief Converts a mesh half-edge to CGAL half-edge index.
 *
 * @param he Mesh half-edge.
 * @return CGAL half-edge index.
 */
MDV_INLINE CgalImpl::CgalHalfEdgeIndex
           to_halfedge_impl(const HalfEdge& he) {
    return CgalImpl::CgalHalfEdgeIndex{he.id()};
}

/**
 * @brief Converts a mesh point to CGAL point.
 *
 * @param pt Mesh point.
 * @return CGAL point.
 */
MDV_INLINE CgalImpl::Kernel::Point_3
           point3_from_point(const Point& pt) {
    return point3_from_eigen(pt.position());
}

/**
 * @brief Converts a tangent vector to a CGAL ray.
 *
 * @param tv Tangent vector.
 * @return CGAL ray.
 */
MDV_INLINE CgalImpl::Kernel::Ray_3
           ray3_from_tangent_vector(const TangentVector& tv) {
    return {point3_from_point(tv.application_point()),
                       direction3_from_eigen(tv.cartesian_vector())};
}

/**
 * @brief Constructs a CGAL triangle from a mesh face.
 *
 * @param f Mesh face.
 * @return CGAL triangle.
 */
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
 * @brief Returns the shared half-edge between two faces if they are adjacent.
 *
 * @param source_face Source face index.
 * @param target_face Target face index.
 * @param mesh CGAL mesh.
 * @return Shared half-edge if adjacent.
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

/**
 * @brief Rotation aligning the normal frames of two adjacent faces.
 *
 * @param he Half-edge shared by the faces.
 * @param mesh CGAL mesh.
 * @return Quaternion aligning face frames.
 */
Eigen::Quaterniond relative_face_rotation(
        const CgalImpl::CgalHalfEdgeIndex& he, const CgalImpl::Mesh& mesh
);

/**
 * @brief Ray-segment intersection on a mesh edge.
 *
 * @param edge CGAL segment.
 * @param ray CGAL ray.
 * @return Intersection point if it exists.
 */
std::optional<CgalImpl::Kernel::Point_3> edge_ray_intersection(
        const CgalImpl::Kernel::Segment_3& edge, const CgalImpl::Kernel::Ray_3& ray
);

/**
 * @brief Total angle around a vertex in radians.
 *
 * @param m CGAL mesh.
 * @param v Vertex descriptor.
 * @return Total angle in radians.
 */
double total_curvature_rad(
        const CgalImpl::Mesh& m, const CgalImpl::VertexDescriptor& v
);

/**
 * @brief Total angle around a vertex in degrees.
 *
 * @param m CGAL mesh.
 * @param v Vertex descriptor.
 * @return Total angle in degrees.
 */
double total_curvature_deg(
        const CgalImpl::Mesh& m, const CgalImpl::VertexDescriptor& v
);

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_CGAL_DATA_HPP
