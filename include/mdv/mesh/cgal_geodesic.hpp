#ifndef MDV_MESH_CGAL_GEODESIC_HPP
#define MDV_MESH_CGAL_GEODESIC_HPP

#include <list>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/point.hpp"

// #define MDV_CACHE_GEODESICS

namespace mdv::mesh::internal {

/**
 * @brief Constructs geodesic polylines using CGAL shortest paths.
 *
 * Provides a cache for shortest-path objects and utilities for building
 * geodesics between points on a mesh, which are key for surface learning tasks.
 */
class CgalGeodesicConstructor {
public:
    using ShortestPath = CgalImpl::ShortestPath;

    using ShortestPathPtr = std::unique_ptr<ShortestPath>;

    /**
     * @brief Constructs a geodesic constructor for a given CGAL mesh.
     *
     * @param mesh CGAL mesh pointer.
     */
    CgalGeodesicConstructor(const CgalImpl::Mesh* mesh) : _reference_mesh(mesh) {};

    /**
     * @brief Builds a geodesic polyline between two points.
     *
     * @param from Start point.
     * @param to End point.
     * @return Geodesic polyline.
     */
    Geodesic operator()(const Point& from, const Point& to);

    /**
     * @brief Sets the source point for a CGAL shortest-path object.
     *
     * @param shpath CGAL shortest-path object.
     * @param source Source point on the mesh.
     */
    static void set_source(ShortestPath& shpath, const Point& source);

    /**
     * @brief Constructs the geodesic polyline to the CGAL source.
     *
     * @param shpath CGAL shortest-path object.
     * @param from Target point.
     * @return Geodesic polyline.
     */
    static Geodesic construct_geodesic(
            ShortestPath& shpath, const ::mdv::mesh::Point& from
    );

    /**
     * @brief Constructs a geodesic polyline with optional reversal.
     *
     * @note: if you know that construct_reversed is always false, then use the overload
     * withouth the construct_reversed flag.
     *
     * @param shpath CGAL shortest-path object.
     * @param from Target point.
     * @param construct_reversed If true, reverses the path.
     * @return Geodesic polyline.
     */
    static Geodesic construct_geodesic(
            ShortestPath&             shpath,
            const ::mdv::mesh::Point& from,
            bool                      construct_reversed
    );

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
