#ifndef MDV_MESH_FWD_HPP
#define MDV_MESH_FWD_HPP

#include <Eigen/Dense>
#include <vector>

namespace mdv::mesh {

class Mesh;

class Vertex;
class Face;
class HalfEdge;

class Point;
class TangentSpace;

class UvMap;

using Index        = long;
using IndexTriplet = std::array<Index, 3>;
// Note: preferred std::array over Eigen::Vector to leverage unpacking features
constexpr Index invalid_index = -1;

using CartesianPoint = Eigen::Vector3d;
using Geodesic       = std::vector<CartesianPoint>;

// Internals
namespace internal {
    class CgalImpl;


}  // namespace internal

}  // namespace mdv::mesh


#endif  // MDV_MESH_FWD_HPP
