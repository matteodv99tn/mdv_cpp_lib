#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/iterator.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <fmt/format.h>

#if __has_include(<CGAL/boost/graph/border.h>)
#include <CGAL/boost/graph/border.h>
#else
#include <CGAL/Polygon_mesh_processing/border.h>
#endif

#include <range/v3/algorithm.hpp>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"

// #define DEBUG_ALGORITHM

namespace mdv::mesh {
namespace pmp = CGAL::Polygon_mesh_processing;
namespace rs  = ::ranges;

Mesh
Mesh::fill_holes(const Mesh& mesh) {
    using HalfedgeId = CgalImpl::CgalHalfEdgeIndex;
    static_assert(std::same_as<CgalImpl::Mesh::Halfedge_index, HalfedgeId>);
    auto m = internal::cgal_mesh(mesh);

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

    // CGAL 6.2 moved boundary extraction out of Polygon_mesh_processing.
#if __has_include(<CGAL/boost/graph/border.h>)
    CGAL::extract_boundary_cycles(m, std::back_inserter(border_cycles));
#else
    pmp::extract_boundary_cycles(m, std::back_inserter(border_cycles));
#endif
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
#if MDV_CGAL_VERSION < 6
        pmp::triangulate_and_refine_hole(
                m,
                border_cycles[i],
                CGAL::Emptyset_iterator(),
                CGAL::Emptyset_iterator()
        );
#else
        pmp::triangulate_and_refine_hole(m, border_cycles[i]);
#endif
    }


    Logger::SharedPtr logger = mesh._logger;
    return Mesh{
            new internal::CgalImpl(std::move(m), std::move(logger)),
            fmt::format("{}-filled", mesh.name())
    };
}

}  // namespace mdv::mesh
