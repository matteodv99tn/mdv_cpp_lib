#ifndef MDV_MESH_ALGORITHM_HPP
#define MDV_MESH_ALGORITHM_HPP

#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"

namespace mdv::mesh {

double length(const Geodesic& geod);

/**
 * @brief Computes the parallel transport of vector v on point p
 *
 */
TangentVector parallel_transport(const TangentVector& v, const Mesh::Point& p);

/**
 * @brief Computes the logarithmic of point "y" w.r.t. to point "p".
 *
 */
TangentVector logarithmic_map(const Mesh::Point& p, const Mesh::Point& y);

/**
 * @brief Computes the exponential map.
 *
 * Note that v already contains information about the position as well as of the vector
 * itself.
 *
 * Optionally, it can yield the geodesic retrieved when "unfolding" the vector v.
 *
 */
Mesh::Point exponential_map(TangentVector v, Geodesic* geod = nullptr);


/**
 * @brief Computes the point-to-face distance
 *
 */
double distance(const Mesh::Face& f, const CartesianPoint& pt);

/**
 * @brief Computes the point-to-point distance
 *
 */
double distance(const Point& p1, const Point& p2);

/**
 * @brief Given two faces, it yields the pair of shared vertices from the 2 faces.
 *
 * This function implicitly assumes that faces are neighbouring. If this is not the
 * case, an exception will be thrown.
 */
std::pair<Mesh::Vertex, Mesh::Vertex> shared_vertices(
        const Mesh::Face& f1, const Mesh::Face& f2
);

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
