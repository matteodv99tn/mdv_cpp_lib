#ifndef MDV_MESH_FWD_HPP
#define MDV_MESH_FWD_HPP

#include <Eigen/Dense>
#include <vector>

namespace mdv::mesh {

class Mesh;
class UvMap;

class Vertex;
class Face;
class Point;
class TangentSpace;

using Index        = long;
using IndexTriplet = std::array<Index, 3>;
// Note: preferred std::array over Eigen::Vector to leverage unpacking features

using CartesianPoint = Eigen::Vector3d;
using Geodesic       = std::vector<CartesianPoint>;

// Constants
constexpr Index invalid_index = -1;

// Internals
namespace internal {

    class MeshData;
    class EigenData;
    class CgalImpl;


}  // namespace internal

}  // namespace mdv::mesh


#endif  // MDV_MESH_FWD_HPP
