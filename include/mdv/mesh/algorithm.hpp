#ifndef MDV_MESH_ALGORITHM_HPP
#define MDV_MESH_ALGORITHM_HPP

#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"

namespace mdv::mesh {

double length(const Geodesic& geod);


/**
 * @brief Retrieves the point at a given normalised curvilinear coordinate "s" on a
 * geodesic.
 *
 * s is assumed to be in the range 0 (beginning of geodesic) to 1 (end of geodesic).
 * To avoid recomputation of the geodesic length, the parameter can be passed as
 * pointer. If not provided, the value is computed.
 *
 */
CartesianPoint point_from_geodesic(
        const Geodesic& geod, double s, const double* len = nullptr
);

/**
 * @brief Resamples a geodesic polyline at some specified coordinates.
 *
 * The coordinate are assumed to be in the range [0, 1]; if a coordinate is outside such
 * range, it gets rounded to the closest admissible value.
 */
Geodesic geodesic_resample(const Geodesic& geod, std::vector<double> coordinates);

/**
 * @brief Computes the parallel transport of vector v on point p
 *
 */
TangentVector parallel_transport(const TangentVector& v, const Point& p);

/**
 * @brief Computes the logarithmic of point "y" w.r.t. to point "p".
 *
 */
TangentVector logarithmic_map(const Point& p, const Point& y);

/**
 * @brief Computes the exponential map.
 *
 * Note that v already contains information about the position as well as of the vector
 * itself.
 *
 * Optionally, it can yield the geodesic retrieved when "unfolding" the vector v.
 *
 */
Point exponential_map(TangentVector v, Geodesic* geod = nullptr);


/**
 * @brief Computes the point-to-face distance
 *
 */
double distance(const Face& f, const CartesianPoint& pt);

/**
 * @brief Computes the point-to-point distance
 *
 */
double distance(const Point& p1, const Point& p2);

/**
 * @brief Computes the edge-to-point distance
 *
 */
double distance(const HalfEdge& he, const Eigen::Vector3d& p);

/**
 * @brief Checks wether the provided UV coordinates are within the "unitary" triangle
 * with vertices
 *   (0, 0)
 *   (1, 0)
 *   (0, 1)
 */
bool uv_in_unitary_triangle(const Eigen::Vector2d& uv);

}  // namespace mdv::mesh


#endif  // MDV_MESH_ALGORITHM_HPP
