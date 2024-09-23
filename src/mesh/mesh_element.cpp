#include <gsl/assert>
#include <stdexcept>

#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh_element.hpp>

using mdv::mesh::Mesh;
using mdv::mesh::MeshElement;

MeshElement::MeshElement(Mesh* mesh) noexcept : _mesh(mesh), _is_modifiable_mesh(true) {
}

MeshElement::MeshElement(const Mesh* mesh) noexcept :
        _mesh(const_cast<Mesh*>(mesh)), _is_modifiable_mesh(false) {
}

Mesh*
MeshElement::mesh() {
    if (!_is_modifiable_mesh) [[unlikely]]
        throw std::domain_error("MeshElement::mesh() called on a const object");
    return _mesh;
}

const Mesh*
MeshElement::cmesh() const noexcept {
    return _mesh;
}

bool
MeshElement::operator==(const MeshElement& other) const noexcept {
    return _mesh == other._mesh;
}

bool
MeshElement::operator!=(const MeshElement& other) const noexcept {
    return _mesh != other._mesh;
}
