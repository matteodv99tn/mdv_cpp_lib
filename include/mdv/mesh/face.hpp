#ifndef MDV_MESH_FACE_HPP
#define MDV_MESH_FACE_HPP

#include <string>

#include "mdv/eigen_defines.hpp"
#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"

namespace mdv::mesh {

/** @brief Legacy mesh-bound holder, retained until Point migrates to handles. */
class Face final : public internal::IndexedMeshElement {
public:
    Face(const Mesh& mesh, Index id) noexcept : IndexedMeshElement(mesh, id) {}

    static Face invalid_face;

    MDV_NODISCARD bool
    operator==(const Face& other) const noexcept {
        return id() == other.id();
    }

    MDV_NODISCARD std::string describe() const override;

private:
    Face() = default;
};

/** @brief Face normal. The handle must belong to mesh. */
MDV_NODISCARD Vec3d normal(const Mesh& mesh, FaceId f);

/** @brief Vertices in CGAL's vertices-around-face order. */
MDV_NODISCARD std::array<VertexId, 3> vertices(const Mesh& mesh, FaceId f);

/** @brief Adjacent faces, including invalid handles at boundary edges. */
MDV_NODISCARD std::array<FaceId, 3> neighbours(const Mesh& mesh, FaceId f);

/** @brief Human-readable face description, preserving the legacy text. */
MDV_NODISCARD std::string describe(const Mesh& mesh, FaceId f);

MDV_NODISCARD FaceId random_face(const Mesh& mesh);

}  // namespace mdv::mesh

#endif  // MDV_MESH_FACE_HPP
