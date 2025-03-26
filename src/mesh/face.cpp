#include "mdv/mesh/face.hpp"

#include <CGAL/boost/graph/iterator.h>
#include <cstdlib>
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

mdv::mesh::UvMap
Face::compute_uv_map() const {
    if (id() == -1) {
        logger().trace("Initialised UV map on undefined face!");
        return {
                {0.0, 0.0, 0.0},
                {0.0, 1.0, 0.0},
                {1.0, 0.0, 0.0}
        };
    }
    return {v1(), v2(), v3()};
}
