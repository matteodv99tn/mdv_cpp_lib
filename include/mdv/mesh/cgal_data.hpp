#ifndef MDV_MESH_CGAL_DATA_HPP
#define MDV_MESH_CGAL_DATA_HPP

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path_traits.h>
#include <filesystem>
#include <optional>


#if MDV_CGAL_VERSION == 5
#include <CGAL/AABB_traits.h>
#elif MDV_CGAL_VERSION == 6
#include <CGAL/AABB_traits_3.h>
#endif

#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/logging.hpp"

namespace mdv::mesh {

//   ____            _ __  __           _
//  / ___|__ _  __ _| |  \/  | ___  ___| |__
// | |   / _` |/ _` | | |\/| |/ _ \/ __| '_ \
// | |__| (_| | (_| | | |  | |  __/\__ \ | | |
//  \____\__, |\__,_|_|_|  |_|\___||___/_| |_|
//       |___/
struct Mesh::CgalData {
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
    using VertexDescriptor = boost::graph_traits<CgalData::Mesh>::vertex_descriptor;

    // Factory functions
    static gsl::owner<CgalData*> from_file(const std::filesystem::path& file_path);

    CgalData(const Mesh&& mesh, const SpdLoggerPtr&& logger, const std::string& name);

    mutable SpdLoggerPtr          _logger;
    Mesh                          _mesh;
    std::unique_ptr<ShortestPath> _shortest_path;
    AabbTree                      _aabb_tree;
    std::string                   _name;

    std::optional<mdv::mesh::Mesh::Point> _current_shortpath_source = std::nullopt;

    void build_vertex_normals_map() noexcept;

    SpdLoggerPtr&
    logger() const {
        return _logger;
    };
};

Mesh::CgalData::FaceLocation location_from_mesh_point(const Mesh::Point& pt) noexcept;

Mesh::Geodesic construct_geodesic(
        Mesh::CgalData::ShortestPath& shpath, const Mesh::Point& from
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

Eigen::Vector3d convert(const Mesh::CgalData::Vec3& x);
Eigen::Vector3d convert(const Mesh::CgalData::Point3& x);

}  // namespace mdv::mesh


#endif  // MDV_MESH_CGAL_DATA_HPP
