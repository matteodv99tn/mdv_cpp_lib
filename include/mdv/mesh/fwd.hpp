#ifndef MDV_MESH_FWD_HPP
#define MDV_MESH_FWD_HPP

#include <Eigen/Dense>
#include <vector>

namespace mdv::mesh {

class Mesh;
class UvMap;

using Point3d  = Eigen::Vector3d;
using Geodesic = std::vector<Point3d>;

}  // namespace mdv::mesh


#endif  // MDV_MESH_FWD_HPP
