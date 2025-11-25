#include "mdv/mesh/cgal_geodesic.hpp"

#include <range/v3/algorithm/transform.hpp>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"

namespace rs = ranges;

namespace mdv::mesh::internal {

namespace {

    void
    set_shpath_obj_source(
            CgalGeodesicConstructor::ShortestPath& shpath, const Point& source_point
    ) {
        if (const auto* pt = source_point.get_as<Point::PointOnVertexDescriptor>()) {
            const Vertex& v = pt->vertex();
            shpath.add_source_point(static_cast<CgalImpl::CgalVertexIndex>(v.id()));
        } else {
            shpath.add_source_point(location_from_mesh_point(source_point));
        }
    }

}  // namespace

Geodesic
CgalGeodesicConstructor::operator()(const Point& from, const Point& to) {
    // sp_obj = shortest path object
    using mdv::condition::are_equal;

#ifdef MDV_CACHE_GEODESICS
    for (const Geodesic& g : _geodesic_cache) {
        if (are_equal(g.front(), from.position()) && are_equal(g.back(), to.position()))
            return g;
    }
    if (_geodesic_cache.size() > 20) _geodesic_cache.clear();
#endif  // MDV_CACHE_GEODESICS

    for (const auto& [pt, sp_obj] : _shortest_path_cache) {
        if (are_equal(from.position(), pt.position()))
            return construct_geodesic(*sp_obj, to, true);
        if (are_equal(to.position(), pt.position()))
            return construct_geodesic(*sp_obj, from, false);
    }

    ShortestPathPtr shpath_from = std::make_unique<ShortestPath>(*_reference_mesh);
    set_shpath_obj_source(*shpath_from, from);
    _shortest_path_cache.emplace_back(from, std::move(shpath_from));

    ShortestPathPtr shpath_to = std::make_unique<ShortestPath>(*_reference_mesh);
    set_shpath_obj_source(*shpath_to, to);
    _shortest_path_cache.emplace_back(to, std::move(shpath_to));

    while (_shortest_path_cache.size() >= 10) _shortest_path_cache.pop_front();

    ShortestPath& sp_obj = *std::get<1>(_shortest_path_cache.back());
    return construct_geodesic(sp_obj, from, false);
}

Geodesic
CgalGeodesicConstructor::construct_geodesic(
        ShortestPath& shpath, const ::mdv::mesh::Point& from, bool construct_reversed
) {
    using VertexDescriptor = Point::PointOnVertexDescriptor;

    std::vector<CgalImpl::Point3> cgal_geod;

    if (const auto* v_desc = from.get_as<VertexDescriptor>()) {
        shpath.shortest_path_points_to_source_points(
                static_cast<CgalImpl::VertexDescriptor>(v_desc->vertex().id()),
                std::back_inserter(cgal_geod)
        );
    } else {
        const auto [face_id, barycentric_coords] = location_from_mesh_point(from);
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

}  // namespace mdv::mesh::internal
