#include "mdv/mesh/sampler.hpp"

#include <cmath>
#include <numbers>
#include <numeric>
#include <stdexcept>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/mesh.hpp"

namespace mdv::mesh {

namespace {
    constexpr double margin = 1e-6;
}

Sampler::Sampler(const Mesh& mesh, std::uint64_t seed) : _mesh(&mesh), _gen(seed) {
    const auto&         m = internal::cgal_mesh(mesh);
    std::vector<double> face_weights;
    std::vector<double> dual_areas(m.num_vertices(), 0.0);
    for (const auto f : m.faces()) {
        const auto   id   = internal::from_cgal(f);
        const auto   vs   = vertices(mesh, id);
        const Vec3d  a    = position(mesh, vs[1]) - position(mesh, vs[0]);
        const Vec3d  b    = position(mesh, vs[2]) - position(mesh, vs[0]);
        const double area = 0.5 * a.cross(b).norm();
        _faces.push_back(id);
        face_weights.push_back(area);
        for (const auto v : vs) dual_areas[v.value] += area / 3.0;
    }
    if (std::accumulate(face_weights.begin(), face_weights.end(), 0.0) <= 0.0)
        throw std::runtime_error("Cannot sample mesh without positive surface area");
    _face_dist = std::discrete_distribution<std::size_t>(
            face_weights.begin(), face_weights.end()
    );

    std::vector<double> edge_weights;
    for (const auto e : m.edges()) {
        auto h = CGAL::halfedge(e, m);
        if (CGAL::is_border(h, m)) h = CGAL::opposite(h, m);
        if (CGAL::is_border(h, m)) continue;
        const auto id = internal::from_cgal(h);
        _edges.push_back(id);
        edge_weights.push_back(direction(mesh, id).norm());
    }
    _edge_dist = std::discrete_distribution<std::size_t>(
            edge_weights.begin(), edge_weights.end()
    );

    std::vector<double> vertex_weights;
    for (const auto v : m.vertices()) {
        _vertices.push_back(internal::from_cgal(v));
        vertex_weights.push_back(dual_areas[v.idx()]);
    }
    _vertex_dist = std::discrete_distribution<std::size_t>(
            vertex_weights.begin(), vertex_weights.end()
    );
}

FaceId
Sampler::face() {
    return _faces[_face_dist(_gen)];
}

PointOnVertex
Sampler::point_on_vertex() {
    return {_vertices[_vertex_dist(_gen)]};
}

PointOnEdge
Sampler::point_on_edge() {
    const auto h = _edges[_edge_dist(_gen)];
    return {.he = h,
            .t  = std::uniform_real_distribution<double>(margin, 1.0 - margin)(_gen)};
}

PointInFace
Sampler::point_in_face() {
    const auto                             f = face();
    std::uniform_real_distribution<double> coords(0.0, 1.0);
    Vec3d                                  b = Vec3d::Zero();
    while ((b.array() < margin).any()) {
        const double s = std::sqrt(coords(_gen));
        const double r = coords(_gen);
        b              = {1.0 - s, s * (1.0 - r), s * r};
    }
    return {
            .f = f, .bary = {b[0], b[1], b[2]}
    };
}

TangentVector
Sampler::unit_tangent(const Point& application_point) {
    if (application_point.get_as<PointInFace>() == nullptr)
        throw std::runtime_error("Unit tangent requires a point inside a face");
    const Vec3d  n = normal(*_mesh, mesh::face(*_mesh, application_point));
    const Vec3d  u = n.unitOrthogonal();
    const Vec3d  v = n.cross(u);
    const double theta =
            std::uniform_real_distribution<double>(0.0, 2.0 * std::numbers::pi)(_gen);
    return {*_mesh, application_point, std::cos(theta) * u + std::sin(theta) * v};
}

std::vector<Point>
Sampler::points_on_vertices(std::size_t n) {
    std::vector<Point> result;
    result.reserve(n);
    for (std::size_t i = 0; i < n; ++i) result.emplace_back(point_on_vertex());
    return result;
}

std::vector<Point>
Sampler::points_on_edges(std::size_t n) {
    std::vector<Point> result;
    result.reserve(n);
    for (std::size_t i = 0; i < n; ++i) result.emplace_back(point_on_edge());
    return result;
}

std::vector<Point>
Sampler::points_in_faces(std::size_t n) {
    std::vector<Point> result;
    result.reserve(n);
    for (std::size_t i = 0; i < n; ++i) result.emplace_back(point_in_face());
    return result;
}

}  // namespace mdv::mesh
