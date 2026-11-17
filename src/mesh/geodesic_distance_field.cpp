#include "mdv/mesh/geodesic_distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "mdv/mesh/cgal_impl.hpp"

namespace mdv::mesh {
namespace {
    using Backend = internal::CgalImpl;

    /**
     * Edge-root expansion reverses t in CGAL 6.2.1. Expand an edge source as
     * exact face roots on both incident faces instead. This is distance-only:
     * these degenerate roots must NOT be used for polyline reconstruction.
     * Keep vertex classification to avoid zero-length visibility rays.
     */
    struct DistanceTraits : Backend::ShortestPathTraits {
        // CGAL requires this exact nested traits type name.
        struct Classify_barycentric_coordinates {  // NOLINT(readability-identifier-naming)

            auto
            operator()(const Barycentric_coordinates& b) const {
                auto result =
                        Backend::ShortestPathTraits::Classify_barycentric_coordinates{}(
                                b
                        );
                if (result.first
                    == CGAL::Surface_mesh_shortest_paths_3::
                            BARYCENTRIC_COORDINATES_ON_BOUNDARY) {
                    result.first = CGAL::Surface_mesh_shortest_paths_3::
                            BARYCENTRIC_COORDINATES_ON_BOUNDED_SIDE;
                }
                return result;
            }
        };

        Classify_barycentric_coordinates
        classify_barycentric_coordinates_object() const {
            return {};
        }
    };

    using DistancePaths = CGAL::Surface_mesh_shortest_path<DistanceTraits>;

    Backend::CgalVertexIndex
    checked_vertex(const Backend::Mesh& mesh, VertexId id) {
        const auto v = internal::to_cgal(id);
        if (!id.is_valid() || !mesh.has_valid_index(v) || mesh.is_removed(v)
            || mesh.halfedge(v) == Backend::Mesh::null_halfedge())
            throw std::runtime_error("Invalid or isolated distance-field vertex");
        // A non-null halfedge can belong to a wire, not to the surface.
        for (const auto f : CGAL::faces_around_target(mesh.halfedge(v), mesh)) {
            if (f != Backend::Mesh::null_face()) return v;
        }
        throw std::runtime_error("Distance-field vertex has no incident face");
    }

    template <typename Function>
    void
    for_each_location(const Backend::Mesh& mesh, const Point& p, const Function& fn) {
        if (const auto* e = p.get_as<PointOnEdge>()) {
            const auto h = internal::to_cgal(e->he);
            if (!mesh.has_valid_index(h) || mesh.is_removed(h))
                throw std::runtime_error("Invalid distance-field halfedge");
            // Do not project through the AABB tree: both twins could then select
            // the same face. Represent this exact edge location in both faces.
            bool has_face = false;
            for (const auto side : {h, opposite(h, mesh)}) {
                const auto f = face(side, mesh);
                if (f != Backend::Mesh::null_face()) {
                    has_face = true;
                    fn(internal::face_location(mesh, p, internal::from_cgal(f)));
                }
            }
            if (!has_face)
                throw std::runtime_error("Distance-field edge has no incident face");
        } else {
            fn(internal::face_location(mesh, p));
        }
    }
}  // namespace

struct GeodesicDistanceField::Impl {
    const Backend::Mesh&                        mesh;
    std::vector<std::unique_ptr<DistancePaths>> trees;

    Impl(const Mesh& m, std::span<const Point> sources) : mesh(internal::cgal_mesh(m)) {
        if (sources.empty())
            throw std::runtime_error("GeodesicDistanceField requires nonempty sources");
        std::unordered_set<VertexId>    vertices;
        std::set<Backend::FaceLocation> locations;
        trees.reserve(sources.size());
        for (const auto& p : sources) {
            auto paths = std::make_unique<DistancePaths>(mesh);
            if (const auto* v = p.get_as<PointOnVertex>()) {
                const auto vertex = checked_vertex(mesh, v->v);
                if (!vertices.insert(v->v).second) continue;
                paths->add_source_point(vertex);
            } else {
                for_each_location(mesh, p, [&](const auto& location) {
                    if (locations.insert(location).second)
                        paths->add_source_point(location);
                });
            }
            if (paths->number_of_source_points() == 0) continue;
            // Native multi-source competition can lose valid face-source
            // windows, even for two interior points on a square. It also
            // asserts for competing boundary vertices. Keep one independent
            // tree per distinct source (both face roots for an edge stay
            // together); queries reuse these trees without building polylines.
            paths->build_sequence_tree();
            trees.push_back(std::move(paths));
        }
    }

    double
    distance(const Point& query) {
        double     result = std::numeric_limits<double>::infinity();
        const auto accept = [&result](const auto& value) {
            // CGAL returns a negative distance when no source is reachable.
            const double d = CGAL::to_double(value.first);
            if (d >= 0.0) result = std::min(result, d);
        };
        if (const auto* v = query.get_as<PointOnVertex>()) {
            const auto vertex = checked_vertex(mesh, v->v);
            for (const auto& tree : trees)
                accept(tree->shortest_distance_to_source_points(vertex));
        } else {
            for_each_location(mesh, query, [&](const auto& location) {
                for (const auto& tree : trees) {
                    accept(tree->shortest_distance_to_source_points(
                            location.first, location.second
                    ));
                }
            });
            if (const auto* e = query.get_as<PointOnEdge>()) {
                const auto   h           = internal::to_cgal(e->he);
                const auto   a           = source(h, mesh);
                const auto   b           = target(h, mesh);
                const double edge_length = std::sqrt(
                        CGAL::to_double(
                                CGAL::squared_distance(mesh.point(a), mesh.point(b))
                        )
                );
                // CGAL can omit a zero-width window along a boundary edge.
                // Paths via either endpoint are valid surface paths and cover
                // that case exactly, without perturbing the edge query.
                for (const auto& tree : trees) {
                    auto da = tree->shortest_distance_to_source_points(a);
                    auto db = tree->shortest_distance_to_source_points(b);
                    if (da.first >= 0) {
                        da.first += e->t * edge_length;
                        accept(da);
                    }
                    if (db.first >= 0) {
                        db.first += (1.0 - e->t) * edge_length;
                        accept(db);
                    }
                }
            }
        }
        return result;
    }
};

GeodesicDistanceField::GeodesicDistanceField(
        const Mesh& mesh, std::span<const Point> sources
) :
        _impl(new Impl(mesh, sources)) {
}

GeodesicDistanceField::~GeodesicDistanceField() {
    delete _impl;
}

GeodesicDistanceField::GeodesicDistanceField(GeodesicDistanceField&& other) noexcept :
        _impl(other._impl) {
    other._impl = nullptr;
}

GeodesicDistanceField&
GeodesicDistanceField::operator=(GeodesicDistanceField&& other) noexcept {
    if (this != &other) {
        delete _impl;
        _impl       = other._impl;
        other._impl = nullptr;
    }
    return *this;
}

double
GeodesicDistanceField::operator()(const Point& query) const {
    if (_impl == nullptr) throw std::runtime_error("Moved-from GeodesicDistanceField");
    return _impl->distance(query);
}

std::vector<double>
GeodesicDistanceField::operator()(std::span<const Point> queries) const {
    if (_impl == nullptr) throw std::runtime_error("Moved-from GeodesicDistanceField");
    std::vector<double> result;
    result.reserve(queries.size());
    for (const auto& query : queries) result.push_back(_impl->distance(query));
    return result;
}

}  // namespace mdv::mesh
