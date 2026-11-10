#include "mdv/mesh/geodesic_distance_field.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/sampler.hpp"

namespace mdv::mesh {
namespace {
    static_assert(!std::is_copy_constructible_v<GeodesicDistanceField>);
    static_assert(!std::is_copy_assignable_v<GeodesicDistanceField>);
    static_assert(std::is_nothrow_move_constructible_v<GeodesicDistanceField>);
    static_assert(std::is_nothrow_move_assignable_v<GeodesicDistanceField>);

    std::vector<Point>
    sample(Sampler& sampler, int kind, std::size_t n) {
        if (kind == 0) return sampler.points_on_vertices(n);
        if (kind == 1) return sampler.points_on_edges(n);
        return sampler.points_in_faces(n);
    }

    Point
    reference_point(const Mesh& mesh, const Point& p) {
        if (p.get_as<PointInFace>() != nullptr) return p;
        const auto& m = internal::cgal_mesh(mesh);
        FaceId      preferred;
        if (const auto* edge = p.get_as<PointOnEdge>()) {
            preferred = face(mesh, edge->he);
            if (!preferred.is_valid()) preferred = face(mesh, twin(mesh, edge->he));
        } else {
            const auto v = internal::to_cgal(p.get_as<PointOnVertex>()->v);
            for (const auto f : CGAL::faces_around_target(m.halfedge(v), m)) {
                if (f != internal::CgalImpl::Mesh::null_face()) {
                    preferred = internal::from_cgal(f);
                    break;
                }
            }
        }
        auto b = internal::face_location(m, p, preferred).second;
        // The legacy path API has edge-root and boundary-vertex bugs. Move
        // only the oracle endpoints into a face interior. The triangle
        // inequality bounds its distance error by the endpoint displacements.
        constexpr double epsilon = 1e-7;
        for (auto& weight : b) weight = (1.0 - epsilon) * weight + epsilon / 3.0;
        return PointInFace{.f = preferred, .bary = b};
    }

    Mesh
    planar_grid(std::size_t side, bool jitter) {
        auto  mesh = Mesh::from_file(config::meshes_directory() / "square.off");
        auto& m    = mesh.cgal()._mesh;
        m.clear();
        std::vector<internal::CgalImpl::CgalVertexIndex> grid;
        for (std::size_t j = 0; j <= side; ++j) {
            for (std::size_t i = 0; i <= side; ++i) {
                double x = static_cast<double>(i) / static_cast<double>(side);
                double y = static_cast<double>(j) / static_cast<double>(side);
                if (jitter && i > 0 && i < side && j > 0 && j < side) {
                    x += 0.1 / static_cast<double>(side)
                         * std::sin(static_cast<double>(13 * i + 7 * j));
                    y += 0.1 / static_cast<double>(side)
                         * std::cos(static_cast<double>(7 * i + 13 * j));
                }
                grid.push_back(m.add_vertex({x, y, 0}));
            }
        }
        for (std::size_t j = 0; j < side; ++j) {
            for (std::size_t i = 0; i < side; ++i) {
                const auto a = grid[j * (side + 1) + i];
                const auto b = grid[j * (side + 1) + i + 1];
                const auto c = grid[(j + 1) * (side + 1) + i + 1];
                const auto d = grid[(j + 1) * (side + 1) + i];
                m.add_face(a, b, c);
                m.add_face(a, c, d);
            }
        }
        mesh.scale(1.0);  // Refresh the spatial index after building the fixture.
        return mesh;
    }

    void
    expect_matches_existing_paths(
            const Mesh&            mesh,
            std::span<const Point> sources,
            std::span<const Point> queries
    ) {
        const GeodesicDistanceField field(mesh, sources);
        std::vector<Point>          reference_sources;
        double                      max_displacement = 0.0;
        for (const auto& source : sources) {
            reference_sources.push_back(reference_point(mesh, source));
            max_displacement = std::max(
                    max_displacement,
                    (position(mesh, reference_sources.back()) - position(mesh, source))
                            .norm()
            );
        }
        const auto batch = field(queries);
        ASSERT_EQ(batch.size(), queries.size());
        for (std::size_t i = 0; i < queries.size(); ++i) {
            SCOPED_TRACE(i);
            const auto reference_query = reference_point(mesh, queries[i]);
            double     expected        = std::numeric_limits<double>::infinity();
            for (std::size_t j = 0; j < sources.size(); ++j) {
                if (same_location(mesh, sources[j], queries[i])) {
                    expected = 0.0;
                    break;
                }
                const auto path =
                        build_geodesic(mesh, reference_sources[j], reference_query);
                ASSERT_FALSE(path.empty());
                expected = std::min(expected, length(path));
            }
            const double tolerance =
                    max_displacement
                    + (position(mesh, reference_query) - position(mesh, queries[i]))
                              .norm()
                    + 1e-9 * std::max(1.0, expected);
            EXPECT_NEAR(batch[i], expected, tolerance);
            EXPECT_EQ(batch[i], field(queries[i]));
        }
        for (const auto d : field(sources)) EXPECT_NEAR(d, 0.0, 1e-9);
        EXPECT_TRUE(field(std::span<const Point>{}).empty());
    }

    using Cases = ::testing::TestWithParam<std::tuple<const char*, int, int>>;

    class GeodesicDistanceFieldPairwise : public Cases {};

    TEST_P(GeodesicDistanceFieldPairwise, MatchesExistingPaths) {
        const auto [name, source_kind, query_kind] = GetParam();
        const auto mesh = Mesh::from_file(config::meshes_directory() / name);
        Sampler    sampler(mesh, 42);
        const auto sources = sample(sampler, source_kind, 20);
        const auto queries = sample(sampler, query_kind, 10);
        expect_matches_existing_paths(mesh, sources, queries);
    }

    INSTANTIATE_TEST_SUITE_P(
            PointKinds,
            GeodesicDistanceFieldPairwise,
            ::testing::Combine(
                    ::testing::Values("square.off", "angle.off"),
                    ::testing::Range(0, 3),
                    ::testing::Range(0, 3)
            ),
            ([](const ::testing::TestParamInfo<Cases::ParamType>& info) {
                const auto [mesh, source, query] = info.param;
                const std::array  kinds{"Vertex", "Edge", "Face"};
                const std::string name =
                        std::string(mesh) == "square.off" ? "Square_" : "Angle_";
                return name + kinds[static_cast<std::size_t>(source)] + "To"
                       + kinds[static_cast<std::size_t>(query)];
            })
    );
}  // namespace

TEST(GeodesicDistanceField, TorusMixedPointKindsMatchExistingPaths) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "torus_simple.off");
    Sampler    sampler(mesh, 42);
    std::vector<Point> sources;
    std::vector<Point> queries;
    // Grids cover the full point-kind matrix; retain one small curved-surface
    // integration case with two vertices, two edge points and two face points.
    for (int kind = 0; kind < 3; ++kind) {
        const auto source_group = sample(sampler, kind, 2);
        const auto query_group  = sample(sampler, kind, 2);
        sources.insert(sources.end(), source_group.begin(), source_group.end());
        queries.insert(queries.end(), query_group.begin(), query_group.end());
    }
    expect_matches_existing_paths(mesh, sources, queries);
}

TEST(GeodesicDistanceField, PlanarAnalyticalDistances) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    Sampler    sampler(mesh, 73);
    auto       sources = sampler.points_on_edges(20);
    auto       queries = sampler.points_on_vertices(10);
    const auto faces   = sampler.points_in_faces(10);
    const auto edges   = sampler.points_on_edges(10);
    queries.insert(queries.end(), faces.begin(), faces.end());
    queries.insert(queries.end(), edges.begin(), edges.end());
    const GeodesicDistanceField field(mesh, sources);
    for (const auto& query : queries) {
        double expected = std::numeric_limits<double>::infinity();
        for (const auto& source : sources) {
            expected = std::min(
                    expected, (position(mesh, query) - position(mesh, source)).norm()
            );
        }
        EXPECT_NEAR(field(query), expected, 1e-12);
    }
}

TEST(GeodesicDistanceField, FoldedSurfaceAnalyticalDistance) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "angle.off");
    // Adjacent perpendicular sides x=1 and y=1; unfold about their shared edge.
    const std::array            sources{from_cartesian(mesh, {1, 0.2, 0.3})};
    const auto                  query = from_cartesian(mesh, {0.4, 1, 0.7});
    const GeodesicDistanceField field(mesh, sources);
    EXPECT_NEAR(field(query), std::hypot(0.8 + 0.6, 0.7 - 0.3), 1e-12);
    EXPECT_GT(
            field(query), (position(mesh, sources[0]) - position(mesh, query)).norm()
    );
}

TEST(GeodesicDistanceField, MixedBoundarySourcesRegression) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    // Reduced from an 80-source mixed sweep (seed 11). A face source followed
    // by a boundary vertex used to make CGAL expand a competing null-face root.
    std::array sources{
            from_cartesian(mesh, {0.34355906, 0.66053075, 0}),
            Point(PointOnVertex{VertexId{1}})
    };
    for (int order = 0; order < 2; ++order) {
        const GeodesicDistanceField field(mesh, sources);
        for (const auto& pos : std::array<Vec3d, 5>{
                     {{0, 1, 0},
                      {0.2, 0.3, 0},
                      {1, 0.14931916756856378, 0},
                      {0.5, 0.5, 0},
                      {1, 0, 0}}
        }) {
            const auto   query    = from_cartesian(mesh, pos);
            const double expected = std::min(
                    (pos - position(mesh, sources[0])).norm(),
                    (pos - position(mesh, sources[1])).norm()
            );
            EXPECT_NEAR(field(query), expected, 1e-12);
        }
        std::swap(sources[0], sources[1]);
    }
}

TEST(GeodesicDistanceField, CompetingFaceSourcesVisibilityRegression) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    // Reduced from a mixed 80-source sweep (seed 87). The native multi-source
    // tree loses the first source's visible window across the square diagonal.
    std::array sources{
            from_cartesian(mesh, {0.2011144033367788, 0.9077176959867589, 0}),
            from_cartesian(mesh, {0.8176273022496866, 0.9883757030499071, 0})
    };
    const auto query =
            from_cartesian(mesh, {0.06841407370350748, 0.8824089295197409, 0});
    const double expected = (position(mesh, query) - position(mesh, sources[0])).norm();
    for (int order = 0; order < 2; ++order) {
        const GeodesicDistanceField field(mesh, sources);
        EXPECT_NEAR(field(query), expected, 1e-12);
        std::swap(sources[0], sources[1]);
    }
}

TEST(GeodesicDistanceField, MixedPointSetsPlanarAnalyticalDistances) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    for (const auto seed : {11U, 87U, 912U}) {
        SCOPED_TRACE(seed);
        Sampler            sampler(mesh, seed);
        std::vector<Point> sources;
        std::vector<Point> queries;
        for (int i = 0; i < 80; ++i) {
            sources.push_back(sample(sampler, i % 3, 1).front());
            queries.push_back(sample(sampler, (i + 1) % 3, 1).front());
        }
        const GeodesicDistanceField field(mesh, sources);
        for (const auto& query : queries) {
            double expected = std::numeric_limits<double>::infinity();
            for (const auto& source : sources) {
                expected = std::min(
                        expected,
                        (position(mesh, query) - position(mesh, source)).norm()
                );
            }
            EXPECT_NEAR(field(query), expected, 1e-12);
        }
    }
}

TEST(GeodesicDistanceField, EdgeRootPreservesParameter) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    // Different incident faces, so a reflected edge source introduces a kink
    // instead of the planar straight-line geodesic.
    for (const auto& a : std::array<Vec3d, 3>{
                 {{0, 0.2, 0}, {0.2, 0.8, 0}, {0.35, 0, 0}}
    }) {
        for (const auto& b : std::array<Vec3d, 3>{
                     {{1, 0.8, 0}, {0.7, 1, 0}, {0.8, 0.2, 0}}
        }) {
            const std::array            sources{from_cartesian(mesh, a)};
            const GeodesicDistanceField field(mesh, sources);
            EXPECT_NEAR(field(from_cartesian(mesh, b)), (b - a).norm(), 1e-12);
        }
    }
}

TEST(GeodesicDistanceField, RegularAndJitteredGridAnalyticalDistances) {
    for (const bool jitter : {false, true}) {
        const auto mesh = planar_grid(jitter ? 8U : 4U, jitter);
        Sampler    sampler(mesh, 87);
        for (int source_kind = 0; source_kind < 3; ++source_kind) {
            const auto                  sources = sample(sampler, source_kind, 20);
            const GeodesicDistanceField field(mesh, sources);
            for (int query_kind = 0; query_kind < 3; ++query_kind) {
                SCOPED_TRACE(
                        ::testing::Message()
                        << jitter << ':' << source_kind << ':' << query_kind
                );
                const auto queries = sample(sampler, query_kind, 10);
                for (const auto& query : queries) {
                    double expected = std::numeric_limits<double>::infinity();
                    for (const auto& source : sources) {
                        expected = std::min(
                                expected,
                                (position(mesh, query) - position(mesh, source)).norm()
                        );
                    }
                    EXPECT_NEAR(field(query), expected, 1e-12);
                }
            }
        }
    }
}

TEST(GeodesicDistanceField, TorusBoundaryWindowRegression) {
    const auto  mesh = Mesh::from_file(config::meshes_directory() / "torus_simple.off");
    const auto& m    = internal::cgal_mesh(mesh);
    const auto  h    = internal::to_cgal(HalfEdgeId{1232});
    constexpr double t     = 0.74075932657906984;
    const Point      query = PointOnEdge{.he = HalfEdgeId{1232}, .t = t};
    const std::array sources{Point(PointOnVertex{internal::from_cgal(source(h, m))})};
    const GeodesicDistanceField field(mesh, sources);
    EXPECT_NEAR(
            field(query),
            (position(mesh, query) - position(mesh, sources[0])).norm(),
            1e-12
    );
    EXPECT_NEAR(
            field(Point(flip(mesh, *query.get_as<PointOnEdge>()))), field(query), 1e-12
    );
}

TEST(GeodesicDistanceField, FoldedVertexDistanceIsNotAmbientChord) {
    const auto       mesh = Mesh::from_file(config::meshes_directory() / "angle.off");
    const std::array sources{Point(PointOnVertex{VertexId{0}})};
    const GeodesicDistanceField field(mesh, sources);
    EXPECT_NEAR(field(PointOnVertex{VertexId{6}}), 2.0, 1e-12);
}

TEST(GeodesicDistanceField, EdgeTwinsAndBoundaryRepresentations) {
    for (const auto* name : {"square.off", "angle.off", "torus_simple.off"}) {
        const auto  mesh = Mesh::from_file(config::meshes_directory() / name);
        const auto& m    = internal::cgal_mesh(mesh);
        Sampler     sampler(mesh, 123);
        const auto  queries = sampler.points_in_faces(10);
        // Reverse trees depend only on the queries, not on the edge under test.
        std::vector<GeodesicDistanceField> reverse_fields;
        reverse_fields.reserve(queries.size());
        for (const auto& q : queries) {
            const std::array q_source{q};
            reverse_fields.emplace_back(mesh, q_source);
        }
        std::size_t checked = 0;
        for (const auto edge : m.edges()) {
            // Keep CI bounded; development sweeps cover much larger sets.
            if (++checked > 12) break;
            const PointOnEdge p{.he = internal::from_cgal(m.halfedge(edge)), .t = 0.3};
            const std::array  a{Point(p)};
            const std::array  b{Point(flip(mesh, p))};
            const GeodesicDistanceField left(mesh, a);
            const GeodesicDistanceField right(mesh, b);
            EXPECT_NEAR(left(a[0]), 0.0, 1e-12);
            EXPECT_NEAR(left(b[0]), 0.0, 1e-12);
            for (std::size_t i = 0; i < queries.size(); ++i) {
                const auto& q       = queries[i];
                const auto& reverse = reverse_fields[i];
                EXPECT_NEAR(left(q), right(q), 1e-12);
                EXPECT_NEAR(reverse(a[0]), reverse(b[0]), 1e-12);
                EXPECT_NEAR(left(q), reverse(a[0]), 1e-9);
            }
        }
    }
}

TEST(GeodesicDistanceField, DuplicatesSourceLifetimeAndMoves) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "angle.off");
    Sampler    sampler(mesh, 99);
    auto       sources = sampler.points_on_edges(2);
    sources.push_back(sources.front());
    const auto            query = sources.front();
    GeodesicDistanceField original(mesh, sources);
    sources.clear();
    GeodesicDistanceField moved(std::move(original));
    EXPECT_NEAR(moved(query), 0.0, 1e-12);
    EXPECT_THROW((void)original(query), std::runtime_error);
    EXPECT_THROW((void)original(std::span<const Point>{}), std::runtime_error);
    const std::array      replacement{Point(PointOnVertex{VertexId{0}})};
    GeodesicDistanceField assigned(mesh, replacement);
    assigned = std::move(moved);
    EXPECT_NEAR(assigned(query), 0.0, 1e-12);
    EXPECT_THROW((void)moved(query), std::runtime_error);
    auto& alias = assigned;
    assigned    = std::move(alias);
    EXPECT_NEAR(assigned(query), 0.0, 1e-12);
    original = std::move(assigned);
    EXPECT_NEAR(original(query), 0.0, 1e-12);
}

TEST(GeodesicDistanceField, RejectsInvalidInputs) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    EXPECT_THROW(
            (GeodesicDistanceField(mesh, std::span<const Point>{})), std::runtime_error
    );
    const std::array            sources{Point(PointOnVertex{VertexId{0}})};
    const GeodesicDistanceField field(mesh, sources);
    for (const auto& p : std::array{
                 Point{},
                 Point(PointOnVertex{
                       VertexId{static_cast<Index>(mesh.num_vertices() + 10U)}
                 }
                 ),
                 Point(PointOnEdge{.he = HalfEdgeId{10000}, .t = 0.5}
                 ),
                 Point(PointInFace{.f = FaceId{10000}, .bary = {0.2, 0.3, 0.5}}
                 )
    }) {
        const std::array invalid{p};
        EXPECT_THROW((GeodesicDistanceField(mesh, invalid)), std::runtime_error);
        EXPECT_THROW((void)field(p), std::runtime_error);
        EXPECT_THROW((void)field(invalid), std::runtime_error);
    }
}

TEST(GeodesicDistanceField, DisconnectedComponents) {
    auto  mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    auto& m    = mesh.cgal()._mesh;
    m.clear();
    const auto                  a        = m.add_vertex({0, 0, 0});
    const auto                  b        = m.add_vertex({1, 0, 0});
    const auto                  c        = m.add_vertex({0, 1, 0});
    const auto                  d        = m.add_vertex({3, 0, 0});
    const auto                  e        = m.add_vertex({4, 0, 0});
    const auto                  f        = m.add_vertex({3, 1, 0});
    const auto                  fa       = m.add_face(a, b, c);
    const auto                  fb       = m.add_face(d, e, f);
    const auto                  isolated = m.add_vertex({9, 9, 9});
    const Point                 pa       = PointOnVertex{internal::from_cgal(a)};
    const Point                 pd       = PointOnVertex{internal::from_cgal(d)};
    const std::array            sources{pa};
    const GeodesicDistanceField field(mesh, sources);
    const std::array            unreachable{
            pd,
            Point(PointOnEdge{.he = internal::from_cgal(m.halfedge(fb)), .t = 0.5               }
            ),
            Point(PointInFace{.f = internal::from_cgal(fb),              .bary = {0.2, 0.3, 0.5}}
            )
    };
    for (const auto value : field(unreachable))
        EXPECT_EQ(value, std::numeric_limits<double>::infinity());
    const Point reachable = PointInFace{
            .f = internal::from_cgal(fa), .bary = {0.2, 0.3, 0.5}
    };
    EXPECT_NEAR(
            field(reachable),
            (position(mesh, reachable) - position(mesh, pa)).norm(),
            1e-12
    );
    const std::array            both{pa, pd};
    const GeodesicDistanceField covered(mesh, both);
    for (const auto value : covered(unreachable)) EXPECT_TRUE(std::isfinite(value));
    const std::array invalid{Point(PointOnVertex{internal::from_cgal(isolated)})};
    EXPECT_THROW((GeodesicDistanceField(mesh, invalid)), std::runtime_error);
    EXPECT_THROW((void)field(invalid[0]), std::runtime_error);
}

}  // namespace mdv::mesh
