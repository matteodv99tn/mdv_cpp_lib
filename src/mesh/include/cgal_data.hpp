#ifndef MDV_MESH_CGAL_DATA_HPP
#define MDV_MESH_CGAL_DATA_HPP

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/aff_transformation_tags.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path_traits.h>
#include <filesystem>
#include <optional>

#include <mdv/mesh/mesh.hpp>
#include <mdv/utils/logging.hpp>

namespace mdv::mesh {

class Mesh::CgalMesh {
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
    using AabbTraits    = CGAL::AABB_traits_3<Kernel, AabbPrimitive>;
    using AabbTree      = CGAL::AABB_tree<AabbTraits>;

    // CGAL typedefs - variable access
    using CgalVertexIndex = Mesh::Vertex_index;
    using CgalFaceIndex   = Mesh::Face_index;
    using FaceLocation    = ShortestPath::Face_location;

    CgalMesh(const std::filesystem::path& file_path, SpdLoggerPtr& logger);

    SpdLoggerPtr                  _logger;
    Mesh                          _mesh;
    std::unique_ptr<ShortestPath> _shortest_path;
    AabbTree                      _aabb_tree;

    std::optional<mdv::mesh::Mesh::Point> _current_shortpath_source = std::nullopt;
};

Mesh::CgalMesh::FaceLocation location_from_mesh_point(const Mesh::Point& pt) noexcept;

Mesh::Geodesic construct_geodesic(
        Mesh::CgalMesh::ShortestPath& shpath, const Mesh::Point& from
);

}  // namespace mdv::mesh


#endif  // MDV_MESH_CGAL_DATA_HPP
