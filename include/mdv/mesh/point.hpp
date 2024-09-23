#ifndef MDV_MESH_POINT_HPP
#define MDV_MESH_POINT_HPP

#include <mdv/eigen_defines.hpp>
#include <mdv/mesh/face.hpp>
#include <mdv/mesh/fwd.hpp>

#include "mdv/macros.hpp"

namespace mdv::mesh {

class Point {
public:
    /**
     * @brief Builds a point on the mesh from a 3D point.
     *
     * The point is built by retrieving the closest point on the mesh.
     *
     * @note Beware that this constructor can rely on the implicit conversion of Mesh*
     * and const Mesh* to MeshElement.
     */
    static Point from_cartesian_position(
            const Point3d_t& point, const MeshElement& mesh
    );

    /**
     * @brief Builds a point on the mesh by specifying the UV coordinates and the face.
     *
     * @note Beware that this constructor can rely on the implicit conversion of Mesh*
     * and const Mesh* to MeshElement.
     */
    Point(const Face& face, const Vec2_t& uv_coords);

    MDV_NODISCARD Point3d_t position() const;

private:
    Face   _face;
    Vec2_t _uv_coords;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_POINT_HPP
