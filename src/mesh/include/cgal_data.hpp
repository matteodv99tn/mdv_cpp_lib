#ifndef MDV_MESH_CGAL_DATA_HPP
#define MDV_MESH_CGAL_DATA_HPP

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
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

#include <mdv/utils/logging.hpp>

namespace mdv::mesh {

class CgalData {
public:
    // CGAL typedefs - general
    using Kernel_t    = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point3_t    = Kernel_t::Point_3;
    using Mesh_t      = CGAL::Surface_mesh<Point3_t>;
    using Transform_t = Kernel_t::Aff_transformation_3;
    using Vec3_t      = Kernel_t::Vector_3;

    // CGAL typedefs - shortest path
    using ShortestPathTraits_t =
            CGAL::Surface_mesh_shortest_path_traits<Kernel_t, Mesh_t>;
    using ShortestPath_t = CGAL::Surface_mesh_shortest_path<ShortestPathTraits_t>;

    // CGAL typedefs - AABB tree
    using AabbPrimitive_t = CGAL::AABB_face_graph_triangle_primitive<Mesh_t>;
    using AabbTraits_t    = CGAL::AABB_traits<Kernel_t, AabbPrimitive_t>;
    using AabbTree_t      = CGAL::AABB_tree<AabbTraits_t>;

    // CGAL typedefs - variable access
    using CgalVertexIndex_t = Mesh_t::Vertex_index;
    using CgalFaceIndex_t   = Mesh_t::Face_index;
    using FaceLocation_t    = ShortestPath_t::Face_location;

    CgalData(const std::filesystem::path& file_path, LoggerPtr_t& logger);

    LoggerPtr_t                     _logger;
    Mesh_t                          _mesh;
    std::unique_ptr<ShortestPath_t> _shortest_path;
    AabbTree_t                      _aabb_tree;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_CGAL_DATA_HPP
