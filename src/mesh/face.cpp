#include "mdv/mesh/face.hpp"

#include <CGAL/boost/graph/iterator.h>
#include <cstdlib>
#include <fmt/format.h>
#include <gsl/assert>
#include <random>
#include <spdlog/spdlog.h>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/vertex.hpp"

using mdv::mesh::Face;
using mdv::mesh::Mesh;
using mdv::mesh::Vertex;

Face
Face::random(const Mesh& m) {
    static std::random_device                  rd;
    static std::mt19937                        gen(rd());
    std::uniform_int_distribution<Face::Index> dist(0, m.num_faces() - 1);
    return {m.data(), dist(gen)};
}

mdv::Vec3d
Face::normal() const {
    const auto& [v0, v1, v2] = vertices();
    const auto e1            = v1.position() - v0.position();
    const auto e2            = v2.position() - v0.position();
    return e1.cross(e2).normalized();
}

Vertex
Face::vertex(const long i) const {
    Expects(i >= 0 && i < 3);
    return {data(), vertices_ids()[i]};
}

Face::VertexTriplet
Face::vertices() const {
    const auto ids = vertices_ids();
    return {Vertex(data(), ids[0]), Vertex(data(), ids[1]), Vertex(data(), ids[2])};
}

std::string
Face::describe() const {
    if (_mesh_data == nullptr) return "Face object of unspecified mesh";
    if (id() == invalid_index)
        return fmt::format("Invalid face on mesh '{}'", data().name);

    const auto [v1, v2, v3] = vertices_ids();
    return fmt::format(
            "Face ID #{} (vertices {}, {}, {}) of mesh '{}'",
            id(),
            v1,
            v2,
            v3,
            data().name
    );
}
