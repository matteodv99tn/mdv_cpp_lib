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
     * @param tree Borrowed tree owned alongside mesh, or null for mesh-only callers.
     */
    CgalGeodesicConstructor(
            const CgalImpl::Mesh* mesh, const CgalImpl::AabbTree* tree = nullptr
    ) :
            _reference_mesh(mesh), _reference_tree(tree) {};

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
    static void set_source(
            const CgalImpl::Mesh&,
            ShortestPath&             shpath,
            const Point&              source,
            const CgalImpl::AabbTree* tree = nullptr
    );

    /**
     * @brief Computes a geodesic in a local shortest path object.
     *
     * This makes the function call thread-safe when called in multi-threaded
     * environment
     *
     * @param from source point of the geodesic
     * @param to target point of the geodesic
     */
    static Geodesic threadlocal_geodesic(
            const CgalImpl::Mesh&,
            const Point&              from,
            const Point&              to,
            const CgalImpl::AabbTree* tree = nullptr
    );

    /**
     * @brief Constructs the geodesic polyline to the CGAL source.
     *
     * @param shpath CGAL shortest-path object.
     * @param from Target point.
     * @return Geodesic polyline.
     */
    static Geodesic construct_geodesic(
            const CgalImpl::Mesh&,
            ShortestPath&             shpath,
            const Point&              from,
            const CgalImpl::AabbTree* tree = nullptr
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
            const CgalImpl::Mesh&     mesh,
            ShortestPath&             shpath,
            const ::mdv::mesh::Point& from,
            bool                      construct_reversed,
            const CgalImpl::AabbTree* tree = nullptr
    );

private:
    using PointShortPathPair = std::tuple<Point, ShortestPathPtr>;
    std::list<PointShortPathPair> _shortest_path_cache;
    const CgalImpl::Mesh*         _reference_mesh = nullptr;
    const CgalImpl::AabbTree*     _reference_tree = nullptr;

#ifdef MDV_CACHE_GEODESICS
    std::vector<Geodesic> _geodesic_cache;
#endif  // MDV_CACHE_GEODESICS
};

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_CGAL_GEODESIC_HPP
