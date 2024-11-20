#include <CGAL/boost/graph/iterator.h>
#include <cstdlib>
#include <gsl/assert>
#include <random>

#include <mdv/eigen_defines.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>

using mdv::mesh::Mesh;

Mesh::Face
Mesh::Face::random(const Mesh& m) noexcept {
    static std::random_device                        rd;
    static std::mt19937                              gen(rd());
    std::uniform_int_distribution<Mesh::Face::Index> dist(0, m.num_faces() - 1);
    return {m, dist(gen)};
}

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
    return {*_mesh, vertices_ids()[i]};
}

Mesh::Face::VertexTriplet
Mesh::Face::vertices() const {
    const auto ids = vertices_ids();
    return {Vertex(*_mesh, ids[0]), Vertex(*_mesh, ids[1]), Vertex(*_mesh, ids[2])};
}
