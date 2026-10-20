#include "mdv/mesh/cgal_geodesic.hpp"

#include <range/v3/algorithm/transform.hpp>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh::internal {
namespace rs = ::ranges;

namespace {
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
    using VertexDescriptor = PointOnVertex;

    std::vector<CgalImpl::Point3> cgal_geod;

    if (const auto* v_desc = from.get_as<VertexDescriptor>()) {
        assert(v_desc->v.value < mesh.num_vertices());
        shpath.shortest_path_points_to_source_points(
                to_cgal(v_desc->v), std::back_inserter(cgal_geod)
        );
    } else {
        const auto [face_id, barycentric_coords] = face_location(mesh, from, tree);
        shpath.shortest_path_points_to_source_points(
                face_id, barycentric_coords, std::back_inserter(cgal_geod)
        );
    }
    if (cgal_geod.size() == 0) { return {}; }

    Geodesic geod(cgal_geod.size());
    auto     to_eigen = [](const auto& pt) -> Eigen::Vector3d { return convert(pt); };
    rs::transform(cgal_geod, geod.begin(), to_eigen);
    return geod;
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
        shpath.shortest_path_points_to_source_points(
                to_cgal(v_desc->v), std::back_inserter(cgal_geod)
        );
    } else {
        const auto [face_id, barycentric_coords] = face_location(mesh, from, tree);
        shpath.shortest_path_points_to_source_points(
                face_id, barycentric_coords, std::back_inserter(cgal_geod)
        );
    }
    if (cgal_geod.size() == 0) { return {}; }

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

        if (length(g1) < length(g2)) return g1;
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
        if (are_equal(point_position(mesh, from), point_position(mesh, pt)))
            return construct_geodesic(mesh, *sp_obj, to, true, _reference_tree);
        if (are_equal(point_position(mesh, to), point_position(mesh, pt)))
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
