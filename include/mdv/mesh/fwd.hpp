#ifndef MDV_MESH_FWD_HPP
#define MDV_MESH_FWD_HPP

#include <Eigen/Dense>
#include <vector>

/**
 * @brief Forward declarations and core type aliases for mdv::mesh.
 *
 * These aliases represent intrinsic geometry types used throughout surface learning and
 * geodesic computations.
 */
namespace mdv::mesh {

/**
 * @brief Discrete surface mesh.
 */
class Mesh;

/**
 * @brief Mesh vertex.
 */
class Vertex;
/**
 * @brief Mesh face.
 */
class Face;
/**
 * @brief Mesh half-edge.
 */
class HalfEdge;

/**
 * @brief Point on a mesh (vertex/edge/face).
 */
class Point;

/**
 * @brief Tangent space on the mesh (forward declaration).
 */
class TangentSpace;

/**
 * @brief Index type for mesh entities.
 */
using Index = unsigned int;

/**
 * @brief Index triplet for triangle faces.
 */
using IndexTriplet = std::array<Index, 3>;
// Note: preferred std::array over Eigen::Vector to leverage unpacking features

/**
 * @brief Sentinel invalid index.
 */
constexpr Index invalid_index = -1;

/**
 * @brief 3D point in ambient space.
 */
using CartesianPoint = Eigen::Vector3d;

/**
 * @brief Geodesic polyline represented as 3D points.
 */
using Geodesic = std::vector<CartesianPoint>;

// Internals
namespace internal {
    /**
     * @brief CGAL implementation detail (PIMPL).
     */
    class CgalImpl;


}  // namespace internal

}  // namespace mdv::mesh


#endif  // MDV_MESH_FWD_HPP
