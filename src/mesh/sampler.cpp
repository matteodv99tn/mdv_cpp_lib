#include "mdv/mesh/sampler.hpp"

#include <stdexcept>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/mesh.hpp"

namespace mdv::mesh {

Sampler::Sampler(const Mesh& mesh, std::uint64_t seed) : _mesh(&mesh), _gen(seed) {
    if (mesh.num_faces() == 0) throw std::runtime_error("Cannot sample empty mesh");
}

FaceId
Sampler::face() {
    return FaceId{std::uniform_int_distribution<
            Index>(0, static_cast<Index>(_mesh->num_faces() - 1))(_gen)};
}

PointOnVertex
Sampler::point_on_vertex() {
    return {VertexId{std::uniform_int_distribution<
            Index>(0, static_cast<Index>(_mesh->num_vertices() - 1))(_gen)}};
}

PointOnEdge
Sampler::point_on_edge() {
    const auto&                          m = internal::cgal_mesh(*_mesh);
    std::uniform_int_distribution<Index> ids(0, m.num_halfedges() - 1);
    HalfEdgeId                           h;
    do { h = HalfEdgeId{ids(_gen)}; } while (CGAL::is_border(internal::to_cgal(h), m));
    std::uniform_real_distribution<double> coords(0.0, 1.0);
    double                                 t;
    do { t = coords(_gen); } while (t == 0.0);
    return {.he = h, .t = t};
}

PointInFace
Sampler::point_in_face() {
    const auto                             f = face();
    std::uniform_real_distribution<double> coords(0.1, 0.9);
    const double                           b0 = coords(_gen);
    const double                           b1 = 0.95 * (1.0 - b0) * coords(_gen);
    return {
            .f = f, .bary = {b0, b1, 1.0 - b0 - b1}
    };
}

TangentVector
Sampler::unit_tangent(const Point& application_point) {
    if (application_point.get_as<PointInFace>() == nullptr)
        throw std::runtime_error("Unit tangent requires a point inside a face");
    const Vec3d n = normal(*_mesh, mesh::face(*_mesh, application_point));
    std::uniform_real_distribution<double> coords(-1.0, 1.0);
    const Vec3d direction{coords(_gen), coords(_gen), coords(_gen)};
    return {*_mesh,
            application_point,
            ((Mat3d::Identity() - n * n.transpose()) * direction).normalized()};
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
