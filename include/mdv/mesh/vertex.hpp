#ifndef MDV_MESH_VERTEX_HPP
#define MDV_MESH_VERTEX_HPP

#include <vector>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"

namespace mdv::mesh {

/**
 * @brief Vertex of a discrete surface mesh.
 *
 * Provides geometric queries (position, normal, curvature) that are commonly
 * used for differential computations on meshes in learning and control.
 *
 * Vertices are view-types on a Mesh object.
 */
class Vertex : public internal::IndexedMeshElement {
public:
    using Vector        = std::vector<Vertex>;
    using Iterator      = Vector::iterator;
    using ConstIterator = Vector::const_iterator;

    /**
     * @brief Constructs a vertex from mesh and index.
     *
     * @param mesh Owning mesh.
     * @param id Vertex index.
     */
    Vertex(const Mesh& mesh, const long id) : IndexedMeshElement(mesh, id) {}

    /**
     * @brief Vertex position in 3D.
     *
     * @return Cartesian position.
     */
    MDV_NODISCARD CartesianPoint position() const noexcept;

    /**
     * @brief Vertex normal (computed by CGAL).
     *
     * @return Unit normal vector.
     */
    MDV_NODISCARD Eigen::Vector3d normal() const noexcept;

    /**
     * @brief Human-readable vertex description.
     *
     * @return Description string.
     */
    MDV_NODISCARD std::string describe() const override;

    /**
     * @brief Total angle around the vertex (radians).
     *
     * @return Sum of incident angles in radians.
     */
    MDV_NODISCARD double total_curvature() const;

    /**
     * @brief Gaussian curvature at the vertex (radians).
     *
     * @return Gaussian curvature (2*pi - total curvature).
     */
    MDV_NODISCARD double gauss_curvature() const;

private:
    friend class Mesh;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_VERTEX_HPP
