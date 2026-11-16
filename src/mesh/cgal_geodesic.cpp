#include "mdv/mesh/cgal_geodesic.hpp"

#include <CGAL/version.h>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <range/v3/algorithm/transform.hpp>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh::internal {
namespace rs = ::ranges;

namespace {
#if CGAL_VERSION_NR < 1050501000
    // Legacy face-root traceback can leave the source triangle and backtrack.
    // Once it enters either triangle incident to the edge source, the chord to
    // that source is a valid surface segment attaining the Euclidean lower bound.
    // Identify entry by simplex handles, never by coincident Cartesian positions.
    struct LegacyPathVisitor {
        const CgalImpl::Mesh&                  mesh;
        CgalGeodesicConstructor::ShortestPath& paths;
        std::vector<CgalImpl::Point3>&         points;
        std::optional<std::size_t>             source_face_hit;

        bool
        is_source_face(CgalImpl::CgalFaceIndex f) const {
            if (!paths.edge_descriptor || f == CgalImpl::Mesh::null_face())
                return false;
            const auto h = to_cgal(paths.edge_descriptor->he);
            return f == face(h, mesh) || f == face(opposite(h, mesh), mesh);
        }

        void
        note_vertex(CgalImpl::CgalVertexIndex v) {
            if (source_face_hit || !paths.edge_descriptor) return;
            const auto h = to_cgal(paths.edge_descriptor->he);
            for (const auto side : {h, opposite(h, mesh)}) {
                if (face(side, mesh) == CgalImpl::Mesh::null_face()) continue;
                if (v == source(side, mesh) || v == target(side, mesh)
                    || v == target(next(side, mesh), mesh))
                    source_face_hit = points.size();
            }
        }

        void
        operator()(CgalImpl::CgalVertexIndex v) {
            note_vertex(v);
            points.push_back(paths.point(v));
        }

        void
        operator()(CgalImpl::CgalHalfEdgeIndex h, double t) {
            if (!source_face_hit && t >= 0 && t <= 1
                && (is_source_face(face(h, mesh))
                    || is_source_face(face(opposite(h, mesh), mesh))))
                source_face_hit = points.size();
            if (t == 0) note_vertex(source(h, mesh));
            else if (t == 1) note_vertex(target(h, mesh));
            points.push_back(paths.point(h, t));
        }

        void
        operator()(
                CgalImpl::CgalFaceIndex                                f,
                const CgalImpl::ShortestPath::Barycentric_coordinates& bary
        ) {
            if (!source_face_hit && is_source_face(f)) source_face_hit = points.size();
            points.push_back(paths.point(f, bary));
        }
    };

    template <typename... Location>
    bool
    append_legacy_path(
            const CgalImpl::Mesh&                  mesh,
            CgalGeodesicConstructor::ShortestPath& paths,
            std::vector<CgalImpl::Point3>&         points,
            const Location&... location
    ) {
        LegacyPathVisitor visitor{mesh, paths, points, {}};
        const auto        result =
                paths.shortest_path_sequence_to_source_points(location..., visitor);
        if (visitor.source_face_hit) {
            points.resize(*visitor.source_face_hit + 2);
            points.back() = *paths.edge_source;
        }
        return result.first >= 0;
    }
#endif

    bool
    improves_path_length(double distance, double best) {
        if (!std::isfinite(best)) return std::isfinite(distance);
        // CGAL's unfolded scalar distance and the reconstructed 3D length use
        // different floating-point operations. Defensively treat a roundoff tie as
        // a tie, rather than tracing a degenerate collinear window. This never
        // changes a mesh point or introduces a geometric boundary tolerance.
        const double roundoff =
                8 * std::numeric_limits<double>::epsilon() * std::max(distance, best);
        return distance + roundoff < best;
    }

    CgalImpl::CgalHalfEdgeIndex
    checked_halfedge(const CgalImpl::Mesh& mesh, const PointOnEdge& e) {
        const auto h = to_cgal(e.he);
        if (!e.he.is_valid() || !mesh.has_valid_index(h) || mesh.is_removed(h))
            throw std::runtime_error("Invalid point halfedge");
        return h;
    }

    bool
    append_vertex_path(
            const CgalImpl::Mesh&                  mesh,
            CgalGeodesicConstructor::ShortestPath& paths,
            CgalImpl::CgalVertexIndex              vertex,
            std::vector<CgalImpl::Point3>&         points
    ) {
        if (paths.edge_descriptor) {
            const auto h = to_cgal(paths.edge_descriptor->he);
            if (vertex == source(h, mesh) || vertex == target(h, mesh)) {
                // Defensive exact shortcut: this segment attains the Euclidean
                // lower bound. CGAL can
                // nevertheless attempt a degenerate collinear traceback.
                points.push_back(mesh.point(vertex));
                points.push_back(*paths.edge_source);
                return true;
            }
        }
#if CGAL_VERSION_NR < 1050501000
        return append_legacy_path(mesh, paths, points, vertex);
#else
        return paths.shortest_path_points_to_source_points(
                            vertex, std::back_inserter(points)
               )
                       .first
               >= 0;
#endif
    }

    CartesianPoint
    point_position(const CgalImpl::Mesh& m, const Point& p) {
        if (const auto* v = p.get_as<PointOnVertex>())
            return convert(m.point(to_cgal(v->v)));
        if (const auto* e = p.get_as<PointOnEdge>()) {
            const auto                    h = to_cgal(e->he);
            const CgalImpl::Kernel::Ray_3 ray(
                    m.point(source(h, m)), m.point(target(h, m))
            );
            return convert(ray.point(e->t));
        }
        const auto& f = *p.get_as<PointInFace>();
        return convert(CgalImpl::ShortestPath::point(to_cgal(f.f), f.bary, m));
    }
}  // namespace

void
CgalGeodesicConstructor::set_source(
        const CgalImpl::Mesh&                  mesh,
        CgalGeodesicConstructor::ShortestPath& shpath,
        const Point&                           source_point,
        const CgalImpl::AabbTree*              tree
) {
    if (const auto* pt = source_point.get_as<PointOnVertex>()) {
        assert(pt->v.value < mesh.num_vertices());
        shpath.add_source_point(to_cgal(pt->v));
    } else if (source_point.get_as<PointOnEdge>() != nullptr) {
        auto location = face_location(mesh, source_point, tree);
        // Since CGAL 5.5, expand_root passes endpoint weights as edge parameters:
        // expand_edge_root calls face_location(he, source_weight), although
        // face_location expects the target weight. Compensate at registration.
        // Keep native EDGE_SOURCE nodes: degenerate FACE_SOURCE roots can
        // produce parallel lines during polyline traceback.
#if CGAL_VERSION_NR >= 1050501000
        std::array<std::size_t, 2> nonzero{};
        std::size_t                j = 0;
        for (std::size_t i = 0; i < 3; ++i) {
            if (location.second[i] != 0) nonzero[j++] = i;
        }
        std::swap(location.second[nonzero[0]], location.second[nonzero[1]]);
#else
        const auto h = checked_halfedge(mesh, *source_point.get_as<PointOnEdge>());
        // CGAL 5.4's native edge roots omit interior propagation windows and
        // can create a fictitious null-face triangle at a border. Exact face
        // roots cover the incident triangles without building reverse trees.
        shpath.legacy_traits.edge_face_roots = true;
#endif
        shpath.edge_source     = point3_from_eigen(point_position(mesh, source_point));
        shpath.edge_descriptor = *source_point.get_as<PointOnEdge>();
        shpath.add_source_point(location);
#if CGAL_VERSION_NR < 1050501000
        for (const auto side : {h, opposite(h, mesh)}) {
            const auto f = face(side, mesh);
            if (f != CgalImpl::Mesh::null_face() && f != location.first)
                shpath.add_source_point(
                        face_location(mesh, source_point, from_cgal(f))
                );
        }
#endif
    } else {
        shpath.add_source_point(face_location(mesh, source_point, tree));
    }
    shpath.build_sequence_tree();
}

Geodesic
CgalGeodesicConstructor::construct_geodesic(
        const CgalImpl::Mesh&     mesh,
        ShortestPath&             shpath,
        const Point&              from,
        const CgalImpl::AabbTree* tree
) {
    return construct_geodesic(mesh, shpath, from, false, tree);
}

Geodesic
CgalGeodesicConstructor::construct_geodesic(
        const CgalImpl::Mesh&     mesh,
        ShortestPath&             shpath,
        const Point&              from,
        bool                      construct_reversed,
        const CgalImpl::AabbTree* tree
) {
    using VertexDescriptor = PointOnVertex;

    std::vector<CgalImpl::Point3> cgal_geod;

    if (const auto* v_desc = from.get_as<VertexDescriptor>()) {
        assert(v_desc->v.value < mesh.num_vertices());
        append_vertex_path(mesh, shpath, to_cgal(v_desc->v), cgal_geod);
    } else if (const auto* e = from.get_as<PointOnEdge>()) {
        const auto h = checked_halfedge(mesh, *e);
        if (shpath.edge_descriptor) {
            const auto source_h = to_cgal(shpath.edge_descriptor->he);
            if (h == source_h || h == opposite(source_h, mesh)) {
                // The shared-edge segment attains the Euclidean lower bound.
                // Do not let collinear CGAL traceback replace it with a detour.
                Geodesic geod{point_position(mesh, from), convert(*shpath.edge_source)};
                if (construct_reversed) std::swap(geod[0], geod[1]);
                return geod;
            }
        }
        double     best   = std::numeric_limits<double>::infinity();
        const auto accept = [&](std::vector<CgalImpl::Point3>&& candidate,
                                bool                            reachable) {
            if (!reachable || candidate.empty()) return;
            if (shpath.edge_source) candidate.back() = *shpath.edge_source;
            // Collinear traceback can backtrack along an edge even when CGAL's
            // distance is correct. Compare the actual paths, so an exact
            // endpoint continuation can replace such a path.
            double d = 0;
            for (std::size_t i = 1; i < candidate.size(); ++i) {
                d += std::sqrt(
                        CGAL::to_double(
                                CGAL::squared_distance(candidate[i - 1], candidate[i])
                        )
                );
            }
            if (d < best) {
                best      = d;
                cgal_geod = std::move(candidate);
            }
        };
        // An edge-aligned path from a pseudo-source may have no face window.
        // Complete it exactly via either endpoint, not an epsilon-offset query.
        // Do this first: tracing an equal-length collinear face window can
        // intersect parallel lines in CGAL even though the vertex path is valid.
        const auto query = point3_from_eigen(point_position(mesh, from));
        std::array endpoints{
                std::pair{source(h, mesh), std::numeric_limits<double>::infinity()},
                std::pair{target(h, mesh), std::numeric_limits<double>::infinity()}
        };
        for (auto& [v, distance] : endpoints) {
            const auto result = shpath.shortest_distance_to_source_points(v);
            if (result.first >= 0) {
                distance = CGAL::to_double(result.first)
                           + std::sqrt(
                                   CGAL::to_double(
                                           CGAL::squared_distance(query, mesh.point(v))
                                   )
                           );
            }
        }
        if (endpoints[1].second < endpoints[0].second)
            std::swap(endpoints[0], endpoints[1]);
        for (const auto& [v, distance] : endpoints) {
            if (!improves_path_length(distance, best)) break;
            std::vector<CgalImpl::Point3> candidate{query};
            const bool reachable = append_vertex_path(mesh, shpath, v, candidate);
            accept(std::move(candidate), reachable);
        }
        for (const auto side : {h, opposite(h, mesh)}) {
            const auto f = face(side, mesh);
            if (f == CgalImpl::Mesh::null_face()) continue;
            const auto location = face_location(mesh, from, from_cgal(f));
            const auto distance = shpath.shortest_distance_to_source_points(
                    location.first, location.second
            );
            if (distance.first < 0
                || !improves_path_length(CGAL::to_double(distance.first), best))
                continue;
            std::vector<CgalImpl::Point3> candidate;
#if CGAL_VERSION_NR < 1050501000
            const bool reachable = append_legacy_path(
                    mesh, shpath, candidate, location.first, location.second
            );
#else
            const bool reachable = shpath.shortest_path_points_to_source_points(
                                                 location.first,
                                                 location.second,
                                                 std::back_inserter(candidate)
                                   )
                                           .first
                                   >= 0;
#endif
            accept(std::move(candidate), reachable);
        }
    } else {
        const auto [face_id, barycentric_coords] = face_location(mesh, from, tree);
#if CGAL_VERSION_NR < 1050501000
        append_legacy_path(mesh, shpath, cgal_geod, face_id, barycentric_coords);
#else
        shpath.shortest_path_points_to_source_points(
                face_id, barycentric_coords, std::back_inserter(cgal_geod)
        );
#endif
    }
    if (shpath.edge_source && !cgal_geod.empty())
        cgal_geod.back() = *shpath.edge_source;

    Geodesic geod(cgal_geod.size());
    auto     to_eigen = [](const auto& pt) -> Eigen::Vector3d { return convert(pt); };
    if (construct_reversed) rs::transform(cgal_geod, geod.rbegin(), to_eigen);
    else rs::transform(cgal_geod, geod.begin(), to_eigen);

#ifdef MDV_CACHE_GEODESICS
    _geodesic_cache.push_back(geod);
#endif  // MDV_CACHE_GEODESICS
    return geod;
}

Geodesic
CgalGeodesicConstructor::operator()(const Point& from, const Point& to) {
    // sp_obj = shortest path object
    using mdv::condition::are_equal;
    const auto& mesh = *_reference_mesh;


    // Handle the special case of edge descriptor that leads to strange behaviors
    using EdgeDescriptor = PointOnEdge;
    if (from.get_as<EdgeDescriptor>() != nullptr
        || to.get_as<EdgeDescriptor>() != nullptr) {
        ShortestPathPtr shpath_from = std::make_unique<ShortestPath>(*_reference_mesh);
        set_source(mesh, *shpath_from, from, _reference_tree);
        Geodesic g1 = construct_geodesic(mesh, *shpath_from, to, true, _reference_tree);

        ShortestPathPtr shpath_to = std::make_unique<ShortestPath>(*_reference_mesh);
        set_source(mesh, *shpath_to, to, _reference_tree);
        Geodesic g2 =
                construct_geodesic(mesh, *shpath_to, from, false, _reference_tree);

        if (g1.empty()) return g2;
        if (g2.empty() || length(g1) < length(g2)) return g1;
        return g2;
    }
#ifdef MDV_CACHE_GEODESICS
    for (const Geodesic& g : _geodesic_cache) {
        if (are_equal(g.front(), point_position(mesh, from))
            && are_equal(g.back(), point_position(mesh, to)))
            return g;
    }
    if (_geodesic_cache.size() > 20) _geodesic_cache.clear();
#endif  // MDV_CACHE_GEODESICS

    for (const auto& [pt, sp_obj] : _shortest_path_cache) {
        if (from == pt)
            return construct_geodesic(mesh, *sp_obj, to, true, _reference_tree);
        if (to == pt)
            return construct_geodesic(mesh, *sp_obj, from, false, _reference_tree);
    }

    ShortestPathPtr shpath_from = std::make_unique<ShortestPath>(*_reference_mesh);
    set_source(mesh, *shpath_from, from, _reference_tree);
    _shortest_path_cache.emplace_back(from, std::move(shpath_from));

    ShortestPathPtr shpath_to = std::make_unique<ShortestPath>(*_reference_mesh);
    set_source(mesh, *shpath_to, to, _reference_tree);
    _shortest_path_cache.emplace_back(to, std::move(shpath_to));

    while (_shortest_path_cache.size() >= 10) _shortest_path_cache.pop_front();

    ShortestPath& sp_obj = *std::get<1>(_shortest_path_cache.back());
    return construct_geodesic(mesh, sp_obj, from, _reference_tree);
}

Geodesic
CgalGeodesicConstructor::threadlocal_geodesic(
        const CgalImpl::Mesh&     mesh,
        const Point&              from,
        const Point&              to,
        const CgalImpl::AabbTree* tree
) {
    ShortestPath shpath_obj(mesh);
    set_source(mesh, shpath_obj, to, tree);
    return construct_geodesic(mesh, shpath_obj, from, tree);
}

}  // namespace mdv::mesh::internal
