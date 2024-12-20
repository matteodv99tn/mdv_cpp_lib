#ifndef MDV_MESH_ALGORITHM_HPP
#define MDV_MESH_ALGORITHM_HPP

#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/tangent_vector.hpp>

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


}  // namespace mdv::mesh


#endif  // MDV_MESH_ALGORITHM_HPP
