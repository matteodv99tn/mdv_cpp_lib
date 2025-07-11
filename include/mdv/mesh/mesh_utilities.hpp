#ifndef MDV_MESH_UTILITIES_HPP
#define MDV_MESH_UTILITIES_HPP

#include <string>

namespace mdv::mesh {

/**
 * @brief Creates a saddle mesh and returns the path where the file has been created
 *
 */
std::string create_saddle();

/**
 * @brief Creates a cone mesh and returns the path where the file has been created
 *
 */
std::string create_cone(std::size_t n_edges = 5, double angle_from_axis_deg = 40.0);


}  // namespace mdv::mesh


#endif  // MDV_MESH_UTILITIES_HPP
