#include <algorithm>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <numbers>
#include <vector>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/geodesic_distance_field.hpp"
#include "mdv/mesh/kernel.hpp"
#include "mdv/mesh/mesh.hpp"

namespace mdv::mesh {
namespace {
    Mesh
    saddle_patch() {
        auto  mesh = Mesh::from_file(config::meshes_directory() / "square.off");
        auto& m    = mesh.cgal()._mesh;
        m.clear();
        // Literal 16-vertex reproduction from mdv_edge_point_distance/REPORT.md.
        const std::array<Vec3d, 16> positions{
                {
                 {0.05478467492858172, 0.1452715689399546, 0.02261108505196222},
                 {0.2412480188388814, 0.01658448809963669, 0.09322270666987285},
                 {0.4830560762411445, -0.08011524378504609, 0.06303071104187799},
                 {0.8066110542114117,
                         -0.03092511152093662,
                         -0.07720467559513226},
                 {0.1253080956801089, 0.1446612017915185, 0.04908914186579135},
                 {0.498435564244422, 0.1830466439331589, 0.04497393508240449},
                 {0.7093209769735386, 0.4015830992107854, 0.001399933090179333},
                 {1.091798624393599, 0.3922091379630334, -0.0001438472700127293},
                 {0.01744999658616915,
                         0.7128207112591681,
                         -0.008349300152339144},
                 {0.5073623028484406, 0.62013768837142, -0.04488119442577678},
                 {0.7930080883152795, 0.865550640982351, 0.06961172801844037},
                 {0.8010954000680592, 0.8590008021771587, 0.07276549109919837},
                 {0.1429617106350277, 1.074216793792278, -0.02645930109983251},
                 {0.1467675634555191, 1.060183710507127, -0.03042963474171442},
                 {0.7585288452386443, 1.075378692228376, 0.02420522961450196},
                 {0.8702622482410236, 0.9555685695916415, 0.07270310697944962},
                 }
        };
        std::vector<internal::CgalImpl::CgalVertexIndex> verts;
        verts.reserve(positions.size());
        for (const auto& p : positions)
            verts.push_back(m.add_vertex({p.x(), p.y(), p.z()}));
        const std::array<std::array<std::size_t, 3>, 18> triangles{
                {
                 {0, 1, 4},
                 {1, 5, 4},
                 {1, 2, 6},
                 {1, 6, 5},
                 {2, 3, 6},
                 {3, 7, 6},
                 {4, 5, 9},
                 {4, 9, 8},
                 {5, 6, 9},
                 {6, 10, 9},
                 {6, 7, 11},
                 {6, 11, 10},
                 {8, 9, 12},
                 {9, 13, 12},
                 {9, 10, 14},
                 {9, 14, 13},
                 {10, 11, 14},
                 {11, 15, 14},
                 }
        };
        for (const auto& f : triangles)
            m.add_face(verts[f[0]], verts[f[1]], verts[f[2]]);
        mesh.scale(1.0);
        return mesh;
    }

    Mesh
    torus() {
        auto  mesh = Mesh::from_file(config::meshes_directory() / "square.off");
        auto& m    = mesh.cgal()._mesh;
        m.clear();
        std::vector<internal::CgalImpl::CgalVertexIndex> verts;
        constexpr std::size_t                            nu = 24;
        constexpr std::size_t                            nv = 12;
        verts.reserve(nu * nv);
        for (std::size_t i = 0; i < nu; ++i) {
            const double u = static_cast<double>(i) * (2 * std::numbers::pi / nu);
            for (std::size_t j = 0; j < nv; ++j) {
                const double v = static_cast<double>(j) * (2 * std::numbers::pi / nv);
                const double r = 1.0 + 0.4 * std::cos(v);
                verts.push_back(m.add_vertex(
                        {r * std::cos(u), r * std::sin(u), 0.4 * std::sin(v)}
                ));
            }
        }
        for (std::size_t i = 0; i < nu; ++i) {
            for (std::size_t j = 0; j < nv; ++j) {
                const auto a = verts[i * nv + j];
                const auto b = verts[((i + 1) % nu) * nv + j];
                const auto c = verts[((i + 1) % nu) * nv + (j + 1) % nv];
                const auto d = verts[i * nv + (j + 1) % nv];
                m.add_face(a, b, c);
                m.add_face(a, c, d);
            }
        }
        mesh.scale(1.0);
        return mesh;
    }

    void
    expect_edge_distances(
            const Mesh&        mesh,
            const Point&       source,
            const PointOnEdge& edge,
            double             expected
    ) {
        const std::vector<Point>    sources{source};
        const GeodesicDistanceField field(mesh, sources);
        const auto                  opposite = flip(mesh, edge);
        std::vector<Point>          queries{
                Point(edge),
                Point(opposite),
                from_cartesian(mesh, position(mesh, Point(edge)))
        };
        // Equivalent arithmetic and Cartesian projection can shift the edge
        // parameter by one ulp; boundary window lookup must remain continuous.
        for (const double toward : {0.0, 1.0}) {
            queries.emplace_back(
                    PointOnEdge{.he = edge.he, .t = std::nextafter(edge.t, toward)}
            );
        }
        // Both incident-face constructors must classify to the same edge position.
        for (const auto& e : {edge, opposite}) {
            const auto f = face(mesh, e.he);
            if (!f.is_valid()) continue;
            const auto bary =
                    internal::face_location(internal::cgal_mesh(mesh), Point(e), f)
                            .second;
            queries.emplace_back(mesh, f, Vec3d{bary[0], bary[1], bary[2]});
            // Raw face points promise strict interior coordinates. Boundary
            // points are supported only through classification or edge handles.
            EXPECT_THROW(
                    (Point(PointInFace{.f = f, .bary = bary})), std::runtime_error
            );
            EXPECT_THROW(
                    (Point(mesh, f, Vec3d{bary[0], bary[1], bary[2]}, false)),
                    std::runtime_error
            );
        }
        MeshKernel        kernel(mesh);
        const auto        distances = kernel.distance_matrix(sources, queries);
        const auto        reversed  = kernel.distance_matrix(queries, sources);
        const auto        batch     = field(queries);
        InexactMeshKernel inexact(mesh);
        const auto        uncached = inexact.distance_matrix(sources, queries);
        inexact.set_points1(sources);
        const auto cached = inexact.distance_matrix(queries);
        for (std::size_t i = 0; i < queries.size(); ++i) {
            SCOPED_TRACE(i);
            const auto& q = queries[i];
            EXPECT_TRUE(same_location(mesh, Point(edge), q));
            const double lower = (position(mesh, source) - position(mesh, q)).norm();
            EXPECT_GE(batch[i], lower - 1e-12);
            EXPECT_NEAR(batch[i], expected, 1e-9);
            EXPECT_NEAR(field(q), batch[i], 1e-12);
            EXPECT_NEAR(distances(0, static_cast<Eigen::Index>(i)), batch[i], 1e-9);
            EXPECT_NEAR(reversed(static_cast<Eigen::Index>(i), 0), batch[i], 1e-9);
            EXPECT_NEAR(uncached(0, static_cast<Eigen::Index>(i)), batch[i], 1e-9);
            EXPECT_NEAR(cached(0, static_cast<Eigen::Index>(i)), batch[i], 1e-9);
            for (const bool reverse : {false, true}) {
                const auto path = reverse ? build_geodesic(mesh, q, source)
                                          : build_geodesic(mesh, source, q);
                ASSERT_GE(path.size(), 2U);
                const auto from = position(mesh, reverse ? q : source);
                const auto to   = position(mesh, reverse ? source : q);
                EXPECT_LE((path.front() - from).norm(), 1e-12);
                EXPECT_LE((path.back() - to).norm(), 1e-12);
                EXPECT_GE(length(path), lower - 1e-12);
                EXPECT_NEAR(length(path), batch[i], 1e-9);
            }
        }
        // Independent continuity oracle: interior queries should converge to the
        // same distance, with error bounded by the displacement inside the face.
        for (const auto& e : {edge, opposite}) {
            const auto f = face(mesh, e.he);
            if (!f.is_valid()) continue;
            const auto bary =
                    internal::face_location(internal::cgal_mesh(mesh), Point(e), f)
                            .second;
            for (const double epsilon : {1e-7, 1e-5}) {
                SCOPED_TRACE(
                        ::testing::Message()
                        << "face=" << f.value << ", epsilon=" << epsilon
                );
                auto moved = bary;
                for (auto& b : moved) b = (1.0 - epsilon) * b + epsilon / 3.0;
                const Point  q = PointInFace{.f = f, .bary = moved};
                const double displacement =
                        (position(mesh, q) - position(mesh, Point(edge))).norm();
                const std::vector<Point> interior{q};
                const auto d = kernel.distance_matrix(sources, interior)(0, 0);
                EXPECT_LE(std::abs(d - batch[0]), displacement + 1e-9);
                EXPECT_NEAR(d, length(build_geodesic(mesh, source, q)), 1e-9);
            }
        }
    }
}  // namespace

TEST(EdgePointDistance, SaddleVertexPathDoesNotDisappear) {
    const auto mesh   = saddle_patch();
    const auto source = from_cartesian(
            mesh, {0.11983881018124183, 0.1280741215155537, 0.04711474306118153}
    );
    const auto& m = internal::cgal_mesh(mesh);
    const auto  h =
            m.halfedge(internal::to_cgal(VertexId{4}), internal::to_cgal(VertexId{8}));
    const PointOnEdge edge{.he = internal::from_cgal(h), .t = 0.75};
    expect_edge_distances(mesh, source, edge, 0.4534408429430705);
}

TEST(EdgePointDistance, TorusDoesNotSelectLongerSaddleVertexPath) {
    const auto  mesh = torus();
    const auto& m    = internal::cgal_mesh(mesh);
    // Pair (21,2147) from the synthetic survey. One-ulp edge-parameter
    // perturbation makes the legacy API select a path of length 0.879568
    // instead of 0.809282: a lower-bound check alone cannot detect this.
    const auto  vs     = vertices(mesh, FaceId{469});
    const Point source = from_cartesian(
            mesh,
            0.3507967079042443 * position(mesh, vs[0])
                    + 0.2105473424307508 * position(mesh, vs[1])
                    + 0.43865594966500493 * position(mesh, vs[2])
    );
    const auto h = m.halfedge(
            internal::to_cgal(VertexId{230}), internal::to_cgal(VertexId{231})
    );
    const PointOnEdge edge{.he = internal::from_cgal(h), .t = 0.75};
    expect_edge_distances(mesh, source, edge, 0.80928188305774906);
}

TEST(EdgePointDistance, EdgeRootsReconstructEndpointsOnSaddlePatch) {
    const auto  mesh = saddle_patch();
    const auto& m    = internal::cgal_mesh(mesh);
    const auto  query_he =
            m.halfedge(internal::to_cgal(VertexId{4}), internal::to_cgal(VertexId{8}));
    const PointOnEdge query{.he = internal::from_cgal(query_he), .t = 0.75};
    // Interior edge roots exercise both incident-face trees. The final root
    // lies across the saddle, so reconstruction cannot use a single-face chord.
    for (const auto& endpoints : std::array<std::array<Index, 2>, 5>{
                 {{1, 5}, {5, 9}, {9, 10}, {0, 1}, {2, 3}}
    }) {
        SCOPED_TRACE(endpoints[0]);
        const auto h = m.halfedge(
                internal::to_cgal(VertexId{endpoints[0]}),
                internal::to_cgal(VertexId{endpoints[1]})
        );
        const PointOnEdge           edge{.he = internal::from_cgal(h), .t = 0.3};
        const std::array            sources{Point(edge)};
        const GeodesicDistanceField field(mesh, sources);
        const double                expected = field(Point(query));
        expect_edge_distances(mesh, Point(edge), query, expected);
        // Reversing a source edge descriptor must not reflect its parameter.
        expect_edge_distances(mesh, Point(flip(mesh, edge)), query, expected);
    }
}

TEST(EdgePointDistance, PlanarEdgeRootsReconstructStraightPaths) {
    const auto mesh   = Mesh::from_file(config::meshes_directory() / "square.off");
    const auto source = from_cartesian(mesh, {0.2, 0.0, 0.0});
    const auto query  = from_cartesian(mesh, {0.8, 1.0, 0.0});
    ASSERT_NE(source.get_as<PointOnEdge>(), nullptr);
    ASSERT_NE(query.get_as<PointOnEdge>(), nullptr);
    expect_edge_distances(
            mesh, source, *query.get_as<PointOnEdge>(), std::hypot(0.6, 1.0)
    );
}

TEST(EdgePointDistance, DisconnectedKernelsReturnInfinityAndZeroCovariance) {
    auto  mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    auto& m    = mesh.cgal()._mesh;
    m.clear();
    for (const double offset : {0.0, 3.0}) {
        const auto a = m.add_vertex({offset, 0, 0});
        const auto b = m.add_vertex({offset + 1, 0, 0});
        const auto c = m.add_vertex({offset, 1, 0});
        m.add_face(a, b, c);
    }
    mesh.scale(1.0);
    // Each side includes a vertex, an edge, and a face point.
    const std::vector<Point> sources{
            PointOnVertex{VertexId{0}},
            from_cartesian(mesh, {0.3, 0, 0}),
            from_cartesian(mesh, {0.2, 0.2, 0})
    };
    const std::vector<Point> queries{
            PointOnVertex{VertexId{3}},
            from_cartesian(mesh, {3.3, 0, 0}),
            from_cartesian(mesh, {3.2, 0.2, 0})
    };
    MeshKernel        exact(mesh);
    InexactMeshKernel inexact(mesh);
    inexact.set_points1(sources);
    for (const auto& distances :
         {exact.distance_matrix(sources, queries),
          inexact.distance_matrix(sources, queries),
          inexact.distance_matrix(queries)}) {
        ASSERT_EQ(distances.rows(), 3);
        ASSERT_EQ(distances.cols(), 3);
        for (Eigen::Index i = 0; i < distances.size(); ++i)
            EXPECT_EQ(distances(i), std::numeric_limits<double>::infinity());
    }
    for (const auto& covariance :
         {exact(sources, queries, 0.5),
          inexact(sources, queries, 0.5),
          inexact(queries, 0.5)}) {
        EXPECT_TRUE(covariance.allFinite());
        EXPECT_TRUE((covariance.array() == 0.0).all());
    }
}

TEST(EdgePointDistance, IdenticalPointKindsHaveZeroDistance) {
    const auto  mesh = saddle_patch();
    const auto& m    = internal::cgal_mesh(mesh);
    const auto  h =
            m.halfedge(internal::to_cgal(VertexId{5}), internal::to_cgal(VertexId{9}));
    const std::vector<Point> points{
            PointOnVertex{VertexId{5}},
            PointOnEdge{.he = internal::from_cgal(h), .t = 0.3},
            PointInFace{.f = FaceId{7}, .bary = {0.2, 0.3, 0.5}}
    };
    MeshKernel        exact(mesh);
    InexactMeshKernel inexact(mesh);
    inexact.set_points1(points);
    const auto a = exact.distance_matrix(points, points);
    const auto b = inexact.distance_matrix(points, points);
    const auto c = inexact.distance_matrix(points);
    for (Eigen::Index i = 0; i < 3; ++i) {
        SCOPED_TRACE(i);
        EXPECT_DOUBLE_EQ(a(i, i), 0.0);
        EXPECT_DOUBLE_EQ(b(i, i), 0.0);
        EXPECT_DOUBLE_EQ(c(i, i), 0.0);
        const auto& p = points[static_cast<std::size_t>(i)];
        EXPECT_DOUBLE_EQ(length(build_geodesic(mesh, p, p)), 0.0);
    }
}

TEST(EdgePointDistance, ShortestPathCacheDistinguishesNearbySources) {
    const auto  mesh = saddle_patch();
    const Point a    = PointInFace{
            .f = FaceId{0},
              .bary = {0.2, 0.3, 0.5}
    };
    const Point nearby = PointInFace{
            .f = FaceId{0},
              .bary = {0.2, 0.3000001, 0.4999999}
    };
    const Point b = PointInFace{
            .f = FaceId{16},
              .bary = {0.2, 0.3, 0.5}
    };
    const Point c = PointInFace{
            .f = FaceId{14},
              .bary = {0.3, 0.2, 0.5}
    };
    const double shift = (position(mesh, a) - position(mesh, nearby)).norm();
    ASSERT_GT(shift, 1e-12);
    ASSERT_LT(shift, 1e-6);
    ASSERT_NE(a, nearby);
    ASSERT_FALSE(build_geodesic(mesh, a, b).empty());
    const auto path = build_geodesic(mesh, nearby, c);
    ASSERT_GE(path.size(), 2U);
    EXPECT_LE((path.front() - position(mesh, nearby)).norm(), 1e-14);
    EXPECT_LE((path.back() - position(mesh, c)).norm(), 1e-14);
}

TEST(EdgePointDistance, FaceLocationsKeepExactBoundaryWeights) {
    const auto  mesh = saddle_patch();
    const auto& m    = internal::cgal_mesh(mesh);
    for (const auto& endpoints : std::array<std::array<Index, 2>, 2>{
                 {{0, 1}, {5, 9}}
    }) {
        const auto h = m.halfedge(
                internal::to_cgal(VertexId{endpoints[0]}),
                internal::to_cgal(VertexId{endpoints[1]})
        );
        const PointOnEdge edge{.he = internal::from_cgal(h), .t = 0.3};
        for (const auto& e : {edge, flip(mesh, edge)}) {
            const Point p(e);
            const auto  check = [&](const internal::CgalImpl::FaceLocation& location) {
                const auto& [f, bary] = location;
                EXPECT_EQ(std::count(bary.begin(), bary.end(), 0.0), 1);
                EXPECT_DOUBLE_EQ(bary[0] + bary[1] + bary[2], 1.0);
                const auto pos = internal::convert(
                        internal::CgalImpl::ShortestPath::point(f, bary, m)
                );
                EXPECT_LE((pos - position(mesh, p)).norm(), 1e-15);
            };
            check(internal::face_location(m, p));
            for (const auto he : {h, opposite(h, m)}) {
                const auto f = face(he, m);
                if (f != internal::CgalImpl::Mesh::null_face())
                    check(internal::face_location(m, p, internal::from_cgal(f)));
            }
        }
    }
    const Point invalid = PointOnEdge{.he = HalfEdgeId{10000}, .t = 0.3};
    EXPECT_THROW((void)internal::face_location(m, invalid), std::runtime_error);
    EXPECT_THROW(
            (void)internal::face_location(m, invalid, FaceId{0}), std::runtime_error
    );
}

TEST(EdgePointDistance, EndpointContinuationBeatsDirectFaceTraceback) {
    const auto                  mesh   = saddle_patch();
    const Point                 source = PointOnEdge{.he = HalfEdgeId{8}, .t = 0.3};
    const Point                 query  = PointOnEdge{.he = HalfEdgeId{4}, .t = 0.3};
    const std::array            sources{source};
    const GeodesicDistanceField field(mesh, sources);
    using Constructor = internal::CgalGeodesicConstructor;
    Constructor::ShortestPath tree(internal::cgal_mesh(mesh));
    Constructor::set_source(internal::cgal_mesh(mesh), tree, source);
    // The endpoint continuation itself must beat the longer, collinearly
    // backtracking face traceback.
    const auto path = Constructor::construct_geodesic(
            internal::cgal_mesh(mesh), tree, query, false, nullptr
    );
    ASSERT_GE(path.size(), 2U);
    EXPECT_NEAR(length(path), field(query), 1e-9);
    EXPECT_LE((path.front() - position(mesh, query)).norm(), 1e-12);
    EXPECT_LE((path.back() - position(mesh, source)).norm(), 1e-12);
}

TEST(EdgePointDistance, SameEdgeShortcutAvoidsCollinearBacktracking) {
    const auto   mesh     = saddle_patch();
    const Point  source   = PointOnEdge{.he = HalfEdgeId{58}, .t = 0.3};
    const Point  query    = PointOnEdge{.he = HalfEdgeId{58}, .t = 0.75};
    const double expected = (position(mesh, query) - position(mesh, source)).norm();
    using Constructor     = internal::CgalGeodesicConstructor;
    Constructor::ShortestPath tree(internal::cgal_mesh(mesh));
    Constructor::set_source(internal::cgal_mesh(mesh), tree, source);
    // Same-edge public paths use a chord shortcut. Exercise the lower-level
    // reconstruction also used by kernels to check its same-edge shortcut.
    const auto path = Constructor::construct_geodesic(
            internal::cgal_mesh(mesh), tree, query, false, nullptr
    );
    ASSERT_GE(path.size(), 2U);
    EXPECT_NEAR(length(path), expected, 1e-9);
    EXPECT_LE((path.front() - position(mesh, query)).norm(), 1e-12);
    EXPECT_LE((path.back() - position(mesh, source)).norm(), 1e-12);
    const std::vector<Point> sources{source};
    const std::vector<Point> queries{query};
    MeshKernel               exact(mesh);
    InexactMeshKernel        inexact(mesh);
    inexact.set_points1(sources);
    EXPECT_NEAR(exact.distance_matrix(sources, queries)(0, 0), expected, 1e-9);
    EXPECT_NEAR(inexact.distance_matrix(sources, queries)(0, 0), expected, 1e-9);
    EXPECT_NEAR(inexact.distance_matrix(queries)(0, 0), expected, 1e-9);
}

}  // namespace mdv::mesh
