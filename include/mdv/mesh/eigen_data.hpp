#ifndef MDV_MESH_EIGEN_DATA_HPP
#define MDV_MESH_EIGEN_DATA_HPP

#include <Eigen/Dense>
#include <vector>

#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh::internal {

/**
 * @class EigenData
 *
 * A Eigen-based container for the mesh, that contains data ported from
 * CgalImplementation.
 */
class EigenData {
public:
    using Vector3d     = Eigen::Vector3d;
    using IndexTriplet = ::mdv::mesh::IndexTriplet;

    /**
     * @brief Vertices of the mesh represented as a vector of 3D vectors.
     * Each element in the vector represents a vertex with coordinates (x, y, z).
     */
    std::vector<Vector3d> vertices;

    /**
     * @brief Faces of the mesh represented as a vector of 3-tuples.
     * Each face is defined by three indices representing the vertices that form it.
     * Assumes a counter-clockwise ordering for normal calculation.
     */
    std::vector<IndexTriplet> faces;

    /**
     * @brief Neighbouring faces of each vertex represented as a vector of 3-tuples.
     * Let v1, v2, v3 be the ordered triplet of vertices of the i-th face, then
     * the i-th triplet from the neighbouring faces contains:
     *   - index 0: id of the face that shares v1 and v2;
     *   - index 1: id of the face that shares v2 and v3;
     *   - index 0: id of the face that shares v3 and v1.
     *
     * If there's no neighbour face, then "invalid_index" can be used.
     */
    std::vector<IndexTriplet> neighbour_faces;
};


}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_EIGEN_DATA_HPP
