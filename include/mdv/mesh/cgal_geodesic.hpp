#ifndef MDV_MESH_CGAL_GEODESIC_HPP
#define MDV_MESH_CGAL_GEODESIC_HPP

#include <list>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/point.hpp"

// #define MDV_CACHE_GEODESICS

namespace mdv::mesh::internal {

class CgalGeodesicConstructor {
public:
    using ShortestPath = CgalImpl::ShortestPath;

    using ShortestPathPtr = std::unique_ptr<ShortestPath>;

    CgalGeodesicConstructor(const CgalImpl::Mesh* mesh) : _reference_mesh(mesh) {};

    Geodesic operator()(const Point& from, const Point& to);

    static void set_source(ShortestPath& shpath, const Point& source);

    static Geodesic construct_geodesic(
            ShortestPath& shpath, const ::mdv::mesh::Point& from
    );

    static Geodesic construct_geodesic(
            ShortestPath&             shpath,
            const ::mdv::mesh::Point& from,
            bool                      construct_reversed
    );  // if construct_reversed = false always, use the overload without the construct
        // reversed flag

private:
    using PointShortPathPair = std::tuple<Point, ShortestPathPtr>;
    std::list<PointShortPathPair> _shortest_path_cache;
    const CgalImpl::Mesh*         _reference_mesh = nullptr;

#ifdef MDV_CACHE_GEODESICS
    std::vector<Geodesic> _geodesic_cache;
#endif  // MDV_CACHE_GEODESICS
};

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_CGAL_GEODESIC_HPP
