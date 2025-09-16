#ifndef MDV_MESH_CGAL_DATA_HPP
#define MDV_MESH_CGAL_DATA_HPP

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path_traits.h>
#include <filesystem>
#include <gsl/pointers>


#if MDV_CGAL_VERSION == 5
#include <CGAL/AABB_traits.h>
#elif MDV_CGAL_VERSION == 6
#include <CGAL/AABB_traits_3.h>
#endif

#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/logging.hpp"

namespace mdv::mesh::internal {

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
    using CgalVertexIndex = Mesh::Vertex_index;
    using CgalFaceIndex   = Mesh::Face_index;
    using FaceLocation    = ShortestPath::Face_location;

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

    mutable Logger::SharedPtr             _logger;
    Mesh                                  _mesh;
    std::unique_ptr<ShortestPath>         _shortest_path;
    AabbTree                              _aabb_tree;
    mutable gsl::owner<mdv::mesh::Point*> _current_shortpath_source = nullptr;

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

/**
 * @brief Constructs a geodesic path from the given source point to the shortest
 *        path source point.
 *
 * @param shpath The shortest path object.
 * @param from The source point.
 * @return Geodesic representing the path from the source point to the current
 *         shortest path source point.
 */
::mdv::mesh::Geodesic construct_geodesic(
        CgalImpl::ShortestPath& shpath, const ::mdv::mesh::Point& from
);

/**
 * @brief Constructs a geodesic path between two points on the mesh.
 *
 * @param cgal_data The CGAL implementation data.
 * @param from The starting point of the geodesic.
 * @param to The ending point of the geodesic.
 * @return Geodesic representing the path between the two points.
 */
::mdv::mesh::Geodesic construct_geodesic(
        const CgalImpl&           cgal_data,
        const ::mdv::mesh::Point& from,
        const ::mdv::mesh::Point& to
);

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

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_CGAL_DATA_HPP
