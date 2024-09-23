#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/vertex.hpp>

#include "cgal_data.hpp"

using mdv::mesh::ConstVertexIterator;
using mdv::mesh::Vertex;
using mdv::mesh::VertexIndex_t;
using mdv::mesh::VertexIterator;

Vertex::Vertex(const MeshElement& m, VertexIndex_t i) noexcept :
        MeshElement(m), _id(i) {
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __ ___
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
// | |  | |  __/ | | | | | |_) |  __/ |  \__ \
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
//

Eigen::Vector3d
Vertex::position() const noexcept {
    const auto& cgal_id     = static_cast<CgalData::Mesh_t::Vertex_index>(_id);
    const auto& cgal_vertex = cmesh()->_cgal_data->_mesh.point(cgal_id);
    return {cgal_vertex.x(), cgal_vertex.y(), cgal_vertex.z()};
}

VertexIndex_t
Vertex::id() const noexcept {
    return _id;
}

//   ___                       _
//  / _ \ _ __   ___ _ __ __ _| |_ ___  _ __ ___
// | | | | '_ \ / _ \ '__/ _` | __/ _ \| '__/ __|
// | |_| | |_) |  __/ | | (_| | || (_) | |  \__ \
//  \___/| .__/ \___|_|  \__,_|\__\___/|_|  |___/
//       |_|

bool
Vertex::operator==(const Vertex& other) const noexcept {
    return MeshElement::operator==(other) && _id == other._id;
}

//  ___ _                 _
// |_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___
//  | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
//  | || ||  __/ | | (_| | || (_) | |  \__ \
// |___|\__\___|_|  \__,_|\__\___/|_|  |___/
//

VertexIterator::VertexIterator(Mesh* mesh, VertexIndex_t index) noexcept :
        _v(mesh, index) {
}

Vertex&
VertexIterator::operator*() noexcept {
    return _v;
}

VertexIterator&
VertexIterator::operator++() noexcept {
    ++_v._id;
    return *this;
}

VertexIterator
VertexIterator::operator++(int) noexcept {
    VertexIterator tmp = *this;
    ++_v._id;
    return tmp;
}

VertexIterator&
VertexIterator::operator--() noexcept {
    --_v._id;
    return *this;
}

VertexIterator
VertexIterator::operator--(int) noexcept {
    VertexIterator tmp = *this;
    --_v._id;
    return tmp;
}

bool
VertexIterator::operator==(const VertexIterator& other) const noexcept {
    return _v == other._v;
}

bool
VertexIterator::operator!=(const VertexIterator& other) const noexcept {
    return !(_v == other._v);
}

ConstVertexIterator::ConstVertexIterator(const Mesh* mesh, VertexIndex_t index) noexcept
        :
        _v(mesh, index) {
}

const Vertex&
ConstVertexIterator::operator*() const noexcept {
    return _v;
}

ConstVertexIterator&
ConstVertexIterator::operator++() noexcept {
    ++_v._id;
    return *this;
}

ConstVertexIterator
ConstVertexIterator::operator++(int) noexcept {
    ConstVertexIterator tmp = *this;
    ++_v._id;
    return tmp;
}

ConstVertexIterator&
ConstVertexIterator::operator--() noexcept {
    --_v._id;
    return *this;
}

ConstVertexIterator
ConstVertexIterator::operator--(int) noexcept {
    ConstVertexIterator tmp = *this;
    --_v._id;
    return tmp;
}

bool
ConstVertexIterator::operator==(const ConstVertexIterator& other) const noexcept {
    return _v == other._v;
}

bool
ConstVertexIterator::operator!=(const ConstVertexIterator& other) const noexcept {
    return !(_v == other._v);
}
