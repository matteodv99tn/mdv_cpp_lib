#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <fmt/base.h>

#include <range/v3/algorithm.hpp>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"

namespace pmp = CGAL::Polygon_mesh_processing;
namespace rs  = ::ranges;

// #define DEBUG_ALGORITHM

namespace mdv::mesh {

Mesh
Mesh::fill_holes(const Mesh& mesh) {
    using HalfedgeId = CgalImpl::CgalHalfEdgeIndex;
    static_assert(std::same_as<CgalImpl::Mesh::Halfedge_index, HalfedgeId>);
    auto m = internal::get_mesh_impl(mesh);

    auto get_boarder_length = [&m](HalfedgeId start_he) -> double {
        double len = 0.0;

        for (auto he : CGAL::halfedges_around_face(start_he, m)) {
            const auto& p1 = m.point(source(he, m));
            const auto& p2 = m.point(target(he, m));
            len += (p1 - p2).squared_length();
        }
        return std::sqrt(len);
    };

    std::vector<HalfedgeId> border_cycles;

    pmp::extract_boundary_cycles(m, std::back_inserter(border_cycles));
    fmt::print("Found {} border cycles\n", border_cycles.size());

    // Sort by length of boarder -> avoid filling of last hole!
    rs::sort(border_cycles, std::less{}, get_boarder_length);

    if (border_cycles.size() <= 1) {
        fmt::print("The mesh appears to have no holes, terminating early!\n");
        Logger::SharedPtr logger = mesh._logger;
        return Mesh{
                new internal::CgalImpl(std::move(m), std::move(logger)),
                fmt::format("{}-filled", mesh.name())
        };
    }

#ifdef DEBUG_ALGORITHM
    for (std::size_t i = 0; i < border_cycles.size(); ++i) {
        const auto   he  = border_cycles[i];
        const double len = get_boarder_length(he);
        fmt::print("  Hole {} - Halfedge {} - length = {}\n", i + 1, he.idx(), len);
    }
#endif

    // Fill all except last borders
    for (std::size_t i = 0; i < border_cycles.size() - 1; ++i) {
        pmp::triangulate_and_refine_hole(m, border_cycles[i]);
    }


    Logger::SharedPtr logger = mesh._logger;
    return Mesh{
            new internal::CgalImpl(std::move(m), std::move(logger)),
            fmt::format("{}-filled", mesh.name())
    };
}

}  // namespace mdv::mesh
