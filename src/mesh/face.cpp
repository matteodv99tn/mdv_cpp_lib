#include <CGAL/boost/graph/iterator.h>
#include <gsl/assert>

#include <mdv/eigen_defines.hpp>
#include <mdv/mesh/face.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/mesh_element.hpp>
#include <mdv/mesh/vertex.hpp>

#include "cgal_data.hpp"

using mdv::mesh::ConstFaceIterator;
using mdv::mesh::Face;
using mdv::mesh::FaceIndex_t;
using mdv::mesh::FaceIterator;
using mdv::mesh::Vertex;

Face::Face(const MeshElement& m, FaceIndex_t i) noexcept : MeshElement(m), _id(i) {
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __ ___
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
// | |  | |  __/ | | | | | |_) |  __/ |  \__ \
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
//

mdv::Vec3_t
Face::normal() const {
    const auto& [v0, v1, v2] = vertices();
    const auto e1            = v1.position() - v0.position();
    const auto e2            = v2.position() - v0.position();
    return e1.cross(e2).normalized();
}

std::array<mdv::mesh::VertexIndex_t, 3>
Face::vertices_ids() const {
#if 0
    const auto face_id_cgal = static_cast<mdv::mesh::CgalData::Mesh_t::Face_index>(_id);
    const CgalData::Mesh_t& cgal_mesh = cmesh()->_cgal_data->_mesh;
    const auto& [vertices_begin, vertices_end] =
            CGAL::vertices_around_face(cgal_mesh.halfedge(face_id_cgal), cgal_mesh);
    std::array<VertexIndex_t, 3> res;
    std::transform(vertices_begin, vertices_end, res.begin(), [](const auto& vertex) {
        return static_cast<VertexIndex_t>(vertex.id());
    });
#else
    const std::array<VertexIndex_t, 3> res{
            cmesh()->_f_mat(0, _id), cmesh()->_f_mat(1, _id), cmesh()->_f_mat(2, _id)
    };
#endif
    return res;
}

Vertex
Face::vertex(const long i) const {
    Expects(i >= 0 && i < 3);
    return {*this, vertices_ids().at(i)};
}

std::array<Vertex, 3>
Face::vertices() const {
    const auto ids = vertices_ids();
    return {
            Vertex{*this, ids[0]},
            Vertex{*this, ids[1]},
            Vertex{*this, ids[2]}
    };
}

FaceIndex_t
Face::id() const {
    return _id;
}

std::array<FaceIndex_t, 3>
Face::neighbour_faces_ids() const {
    // Function that checks if an id is contained in an array
    auto contains_id = [](const std::array<FaceIndex_t, 3>& arr,
                          const FaceIndex_t&                id) {
        return std::any_of(arr.begin(), arr.end(), [id](const auto& i) {
            return i == id;
        });
    };

    std::array<FaceIndex_t, 3> res{-1, -1, -1};
    const auto [this_v0, this_v1, this_v2] = vertices_ids();

#if 0
    for (const Face& other : cmesh()->cfaces()) {
        if (other == *this) continue;  // Skip this face

        // Check for neigbours
        const auto other_vertices = other.vertices_ids();
        if (contains_id(other_vertices, this_v0)
            && contains_id(other_vertices, this_v1)) {
            assert(res[0] == -1);
            res[0] = other.id();
        }
        if (contains_id(other_vertices, this_v1)
            && contains_id(other_vertices, this_v2)) {
            assert(res[1] == -1);
            res[1] = other.id();
        }
        if (contains_id(other_vertices, this_v2)
            && contains_id(other_vertices, this_v0)) {
            assert(res[2] == -1);
            res[2] = other.id();
        }
    }
#else
    for (auto it = cmesh()->faces_begin(); it != cmesh()->faces_end(); ++it) {
        if ((*it) == *this) continue;  // Skip this face

        // Check for neigbours
        const auto other_vertices = (*it).vertices_ids();
        if (contains_id(other_vertices, this_v0)
            && contains_id(other_vertices, this_v1)) {
            assert(res[0] == -1);
            res[0] = (*it).id();
        }
        if (contains_id(other_vertices, this_v1)
            && contains_id(other_vertices, this_v2)) {
            assert(res[1] == -1);
            res[1] = (*it).id();
        }
        if (contains_id(other_vertices, this_v2)
            && contains_id(other_vertices, this_v0)) {
            assert(res[2] == -1);
            res[2] = (*it).id();
        }
    }
#endif
    return res;
}

//   ___                       _
//  / _ \ _ __   ___ _ __ __ _| |_ ___  _ __ ___
// | | | | '_ \ / _ \ '__/ _` | __/ _ \| '__/ __|
// | |_| | |_) |  __/ | | (_| | || (_) | |  \__ \
//  \___/| .__/ \___|_|  \__,_|\__\___/|_|  |___/
//       |_|

bool
Face::operator==(const Face& other) const noexcept {
    return MeshElement::operator==(other) && _id == other._id;
}

//  ___ _                 _
// |_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___
//  | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
//  | || ||  __/ | | (_| | || (_) | |  \__ \
// |___|\__\___|_|  \__,_|\__\___/|_|  |___/
//

FaceIterator::FaceIterator(Mesh* mesh, FaceIndex_t index) noexcept : _v(mesh, index) {
}

Face&
FaceIterator::operator*() noexcept {
    return _v;
}

FaceIterator&
FaceIterator::operator++() noexcept {
    ++_v._id;
    return *this;
}

FaceIterator
FaceIterator::operator++(int) noexcept {
    FaceIterator tmp = *this;
    ++_v._id;
    return tmp;
}

FaceIterator&
FaceIterator::operator--() noexcept {
    --_v._id;
    return *this;
}

FaceIterator
FaceIterator::operator--(int) noexcept {
    FaceIterator tmp = *this;
    --_v._id;
    return tmp;
}

bool
FaceIterator::operator==(const FaceIterator& other) const noexcept {
    return _v == other._v;
}

bool
FaceIterator::operator!=(const FaceIterator& other) const noexcept {
    return !(_v == other._v);
}

ConstFaceIterator::ConstFaceIterator(const Mesh* mesh, FaceIndex_t index) noexcept :
        _v(mesh, index) {
}

const Face&
ConstFaceIterator::operator*() const noexcept {
    return _v;
}

ConstFaceIterator&
ConstFaceIterator::operator++() noexcept {
    ++_v._id;
    return *this;
}

ConstFaceIterator
ConstFaceIterator::operator++(int) noexcept {
    ConstFaceIterator tmp = *this;
    ++_v._id;
    return tmp;
}

ConstFaceIterator&
ConstFaceIterator::operator--() noexcept {
    --_v._id;
    return *this;
}

ConstFaceIterator
ConstFaceIterator::operator--(int) noexcept {
    ConstFaceIterator tmp = *this;
    --_v._id;
    return tmp;
}

bool
ConstFaceIterator::operator==(const ConstFaceIterator& other) const noexcept {
    return _v == other._v;
}

bool
ConstFaceIterator::operator!=(const ConstFaceIterator& other) const noexcept {
    return !(_v == other._v);
}
