#include <CGAL/boost/graph/iterator.h>
#include <gsl/assert>

#include <mdv/eigen_defines.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>

using mdv::mesh::Mesh;

mdv::Vec3d
Mesh::Face::normal() const {
    const auto& [v0, v1, v2] = vertices();
    const auto e1            = v1.position() - v0.position();
    const auto e2            = v2.position() - v0.position();
    return e1.cross(e2).normalized();
}

Mesh::Vertex
Mesh::Face::vertex(const long i) const {
    Expects(i >= 0 && i < 3);
    return {this->_mesh, vertices_ids()[i]};
}

Mesh::Face::VertexTriplet
Mesh::Face::vertices() const {
    const auto ids = vertices_ids();
    return {Vertex(this->_mesh, ids[0]),
            Vertex(this->_mesh, ids[1]),
            Vertex(this->_mesh, ids[2])};
}
