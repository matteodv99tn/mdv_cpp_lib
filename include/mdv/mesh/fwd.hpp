#ifndef MDV_MESH_FWD_HPP
#define MDV_MESH_FWD_HPP

#include <Eigen/Dense>

namespace mdv::mesh {

class Mesh;
class Vertex;
class Face;
class Edge;

class MeshPoint;

/**
 * @brief Hidden class for internal use.
 *
 * To avoid directly exposing the CGAL library which slows down compilation, we define
 * this opaque struct.
 */
class CgalData;

using VertexIndex_t = long;
using FaceIndex_t   = long;

using Point3d_t = Eigen::Vector3d;

}  // namespace mdv::mesh


#endif  // MDV_MESH_FWD_HPP
