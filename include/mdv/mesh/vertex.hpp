#ifndef MDV_MESH_VERTEX_HPP
#define MDV_MESH_VERTEX_HPP

#include <vector>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"

namespace mdv::mesh {

class Vertex : public internal::IndexedMeshElement {
public:
    using Vector        = std::vector<Vertex>;
    using Iterator      = Vector::iterator;
    using ConstIterator = Vector::const_iterator;

    Vertex(const Mesh& mesh, const long id) : IndexedMeshElement(mesh, id) {}

    MDV_NODISCARD CartesianPoint position() const noexcept;

    MDV_NODISCARD Eigen::Vector3d normal() const noexcept;

    MDV_NODISCARD std::string describe() const override;

private:
    friend class Mesh;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_VERTEX_HPP
