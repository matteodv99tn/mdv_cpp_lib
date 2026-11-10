#ifndef MDV_MESH_GEODESIC_DISTANCE_FIELD_HPP
#define MDV_MESH_GEODESIC_DISTANCE_FIELD_HPP

#include <gsl/pointers>
#include <span>
#include <vector>

#include "mdv/macros.hpp"
#include "mdv/mesh/point.hpp"

namespace mdv::mesh {

/**
 * @brief Minimum surface-geodesic distance to a fixed set of mesh points.
 *
 * Precomputes independent shortest-path trees, without exposing CGAL types.
 * One tree per distinct source avoids CGAL's multi-source visibility/boundary
 * bugs. Queries take the minimum over the trees without constructing polylines;
 * query work and cached storage scale with the number of distinct sources.
 * The mesh is borrowed and must outlive the field. Do not change its geometry or
 * topology while the field exists; reconstruct the field after such changes.
 * Source storage is not borrowed. Points must refer to the supplied mesh; mesh
 * membership cannot be inferred from mesh-free handles.
 * Isolated vertices and wire edges are not surface locations and are rejected.
 *
 * Queries on the same field must not run concurrently, including const queries.
 * The field is movable, not copyable. A moved-from field may only be destroyed
 * or assigned to; querying it throws std::runtime_error.
 */
class GeodesicDistanceField {
public:
    /**
     * @brief Build a distance field from nonempty sources on mesh.
     * @throws std::runtime_error for empty sources or invalid mesh handles.
     */
    GeodesicDistanceField(const Mesh& mesh, std::span<const Point> sources);
    ~GeodesicDistanceField();

    GeodesicDistanceField(const GeodesicDistanceField&)            = delete;
    GeodesicDistanceField& operator=(const GeodesicDistanceField&) = delete;
    GeodesicDistanceField(GeodesicDistanceField&& other) noexcept;
    GeodesicDistanceField& operator=(GeodesicDistanceField&& other) noexcept;

    /**
     * @brief Distance along the surface to the nearest source.
     * @return Nonnegative distance, or positive infinity if no source is reachable.
     * @throws std::runtime_error for an invalid query or a moved-from field.
     */
    MDV_NODISCARD double operator()(const Point& query) const;

    /** @brief Evaluate queries in input order; an empty batch returns an empty vector.
     * @throws std::runtime_error for an invalid query or a moved-from field.
     */
    MDV_NODISCARD std::vector<double> operator()(std::span<const Point> queries) const;

private:
    struct Impl;
    gsl::owner<Impl*> _impl = nullptr;
};

}  // namespace mdv::mesh

#endif  // MDV_MESH_GEODESIC_DISTANCE_FIELD_HPP
