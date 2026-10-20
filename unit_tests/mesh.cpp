#include "mdv/mesh/mesh.hpp"

#include <algorithm>
#include <gtest/gtest.h>
#include <numbers>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/sampler.hpp"

namespace mdv::mesh {
namespace fs = std::filesystem;

TEST(MeshGeodesic, ResamplePreservesCoordinateOrderAndClampsEndpoints) {
    const Geodesic geod{
            {0.0, 0.0, 0.0},
            {2.0, 0.0, 0.0},
            {2.0, 2.0, 0.0}
    };
    const std::vector<double> coordinates{1.5, 0.25, -0.5, 0.75};
    const Geodesic            expected{
            {2.0, 2.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {2.0, 1.0, 0.0}
    };

    const auto resampled = geodesic_resample(geod, coordinates);
    ASSERT_EQ(resampled.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_NEAR((resampled[i] - expected[i]).norm(), 0.0, 1e-12);
    }
    EXPECT_TRUE(geodesic_resample(geod, std::vector<double>{}).empty());
}

TEST(MeshGeodesic, RandomPoints) {
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);
    Sampler        sampler(mesh, 42);

    for (std::size_t i = 0; i < 40; ++i) {
        const auto p0 = sampler.point_in_face();
        const auto p1 = sampler.point_in_face();

        const auto g0 = build_geodesic(mesh, p0, p1);
        const auto g1 = build_geodesic(mesh, p1, p0);
        EXPECT_NEAR(length(g0), length(g1), 1e-9);
    }
}

TEST(MeshGeodesic, RandomVertexPoints) {
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);

    Sampler    sampler(mesh, 42);
    const auto points = sampler.points_on_vertices(80);
    for (std::size_t i = 0; i < 40; ++i) {
        const auto& p0 = points[2 * i];
        const auto& p1 = points[2 * i + 1];

        const auto g0 = build_geodesic(mesh, p0, p1);
        const auto g1 = build_geodesic(mesh, p1, p0);
        EXPECT_NEAR(length(g0), length(g1), 1e-9)
                << "p0: " << describe(mesh, p0) << "\np1: " << describe(mesh, p1);
    }
}

TEST(MeshGeodesic, RandomEdgePoints) {
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);

    Sampler sampler(mesh, 42);
    for (std::size_t i = 0; i < 40; ++i) {
        const Point p0 = sampler.point_on_edge();
        const Point p1 = sampler.point_on_edge();

        const auto g0 = build_geodesic(mesh, p0, p1);
        const auto g1 = build_geodesic(mesh, p1, p0);

        ASSERT_NEAR(length(g0), length(g1), 1e-9)
                << "Iteration " << i << "\np0: " << describe(mesh, p0)
                << "\np1: " << describe(mesh, p1);
    }
}

TEST(Vertex, Curvatures) {
    const fs::path mesh_path = mdv::config::meshes_directory() / "cube.stl";
    const auto     mesh      = Mesh::from_file(mesh_path);

    for (Index i = 0; i < mesh.num_vertices(); ++i) {
        ASSERT_FLOAT_EQ(
                static_cast<float>(total_angle(mesh, VertexId{i})),
                static_cast<float>(1.5 * M_PI)
        );
        ASSERT_FLOAT_EQ(
                static_cast<float>(gauss_curvature(mesh, VertexId{i})),
                static_cast<float>(0.5 * M_PI)
        );
    }
}

TEST(MeshGeodesic, DeterministicDescriptorInvariants) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "torus_simple.off");
    const FaceId      f{0};
    const PointOnEdge edge{.he = halfedge(mesh, vertices(mesh, f)[0], f), .t = 0.3};
    const std::vector<Point> points{
            Point(PointOnVertex{VertexId{0}}),
            Point(PointOnVertex{VertexId{17}}),
            Point(edge),
            Point(flip(mesh, edge)),
            Point(mesh, FaceId{0}, {0.2, 0.3, 0.5}),
            Point(mesh, FaceId{23}, {0.4, 0.35, 0.25})
    };
    for (std::size_t i = 0; i < points.size(); ++i) {
        SCOPED_TRACE(i);
        const auto& from      = points[i];
        const auto  projected = from_cartesian(mesh, position(mesh, from));
        EXPECT_NEAR(
                (position(mesh, projected) - position(mesh, from)).norm(), 0.0, 1e-9
        );
        EXPECT_NEAR(length(build_geodesic(mesh, from, from)), 0.0, 1e-9);
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            SCOPED_TRACE(j);
            const auto& to      = points[j];
            const auto  forward = build_geodesic(mesh, from, to);
            const auto  reverse = build_geodesic(mesh, to, from);
            if ((position(mesh, to) - position(mesh, from)).norm() < 1e-12) {
                // Coincident positions (including opposite edge descriptors) have
                // an empty polyline in the current API.
                EXPECT_NEAR(length(forward), 0.0, 1e-9);
                EXPECT_NEAR(length(reverse), 0.0, 1e-9);
                continue;
            }
            ASSERT_FALSE(forward.empty());
            ASSERT_FALSE(reverse.empty());
            EXPECT_NEAR((forward.front() - position(mesh, from)).norm(), 0.0, 1e-9);
            EXPECT_NEAR((forward.back() - position(mesh, to)).norm(), 0.0, 1e-9);
            EXPECT_NEAR((reverse.front() - position(mesh, to)).norm(), 0.0, 1e-9);
            EXPECT_NEAR((reverse.back() - position(mesh, from)).norm(), 0.0, 1e-9);
            EXPECT_NEAR(length(forward), length(reverse), 1e-9);
            EXPECT_GE(
                    length(forward) + 1e-9,
                    (position(mesh, to) - position(mesh, from)).norm()
            );
        }
    }
}

TEST(Vertex, ClosedMeshCurvatureSums) {
    for (const auto* name : {"cube.stl", "torus_simple.off"}) {
        SCOPED_TRACE(name);
        const auto mesh        = Mesh::from_file(config::meshes_directory() / name);
        double     total_angle = 0.0;
        double     curvature   = 0.0;
        for (Index i = 0; i < mesh.num_vertices(); ++i) {
            total_angle += mdv::mesh::total_angle(mesh, VertexId{i});
            curvature += gauss_curvature(mesh, VertexId{i});
        }
        // A closed triangular mesh has E = 3F/2.
        const double euler = static_cast<double>(mesh.num_vertices())
                             - 0.5 * static_cast<double>(mesh.num_faces());
        EXPECT_NEAR(
                total_angle,
                std::numbers::pi * static_cast<double>(mesh.num_faces()),
                1e-9
        );
        EXPECT_NEAR(curvature, 2.0 * std::numbers::pi * euler, 1e-9);
    }
}

TEST(Vertex, HandleQueriesSurviveMeshMove) {
    auto           mesh = Mesh::from_file(config::meshes_directory() / "cube.stl");
    const VertexId v{0};
    const CartesianPoint p     = position(mesh, v);
    const Vec3d          n     = normal(mesh, v);
    const double         angle = total_angle(mesh, v);
    EXPECT_EQ(closest_vertex(mesh, p), v);
    EXPECT_EQ(describe(mesh, VertexId{}), "Invalid Vertex");
    auto moved = std::move(mesh);
    EXPECT_NEAR((position(moved, v) - p).norm(), 0.0, 1e-12);
    EXPECT_NEAR((normal(moved, v) - n).norm(), 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(total_angle(moved, v), angle);
    EXPECT_EQ(closest_vertex(moved, p), v);
}

TEST(Face, EqualityUsesElementIndex) {
    const auto mesh  = Mesh::from_file(config::meshes_directory() / "square.off");
    const auto first = FaceId{0};
    const auto copy  = FaceId{0};
    EXPECT_EQ(first, copy);
    EXPECT_EQ(FaceId{0}, FaceId{0});
    EXPECT_NE(FaceId{0}, FaceId{1});
    EXPECT_TRUE(Point{}.is_undefined());
    EXPECT_FALSE(Point{PointOnVertex{VertexId{0}}}.is_undefined());
    const Point interior{
            PointInFace{.f = first, .bary = {0.2, 0.3, 0.5}}
    };
    EXPECT_FALSE(interior.is_undefined());
}

TEST(Face, HandleTopologyAndBoundaryOrdering) {
    for (const auto* name : {"cube.stl", "torus_simple.off", "square.off"}) {
        SCOPED_TRACE(name);
        const auto  mesh     = Mesh::from_file(config::meshes_directory() / name);
        const auto& m        = internal::cgal_mesh(mesh);
        std::size_t boundary = 0;
        for (Index i = 0; i < mesh.num_faces(); ++i) {
            const FaceId f{i};
            const auto   vs = vertices(mesh, f);
            const auto   ns = neighbours(mesh, f);
            EXPECT_NE(vs[0], vs[1]);
            EXPECT_NE(vs[1], vs[2]);
            EXPECT_NE(vs[2], vs[0]);
            auto h = CGAL::halfedge(internal::to_cgal(f), m);
            for (std::size_t j = 0; j < 3; ++j) {
                EXPECT_EQ(vs[j], internal::from_cgal(CGAL::target(h, m)));
                EXPECT_EQ(
                        ns[j], internal::from_cgal(CGAL::face(CGAL::opposite(h, m), m))
                );
                if (ns[j].is_valid()) {
                    const auto adjacent = neighbours(mesh, ns[j]);
                    EXPECT_NE(std::ranges::find(adjacent, f), adjacent.end());
                } else {
                    EXPECT_EQ(ns[j], FaceId{});
                    ++boundary;
                }
                h = CGAL::next(h, m);
            }
            const Vec3d a = position(mesh, vs[1]) - position(mesh, vs[0]);
            const Vec3d b = position(mesh, vs[2]) - position(mesh, vs[0]);
            EXPECT_NEAR((a.cross(b).normalized() - normal(mesh, f)).norm(), 0.0, 1e-12);
        }
        EXPECT_EQ(boundary, std::string_view{name} == "square.off" ? 4U : 0U);
        EXPECT_EQ(describe(mesh, FaceId{}), "Invalid face object");
        EXPECT_LT(Sampler(mesh, 42).face().value, mesh.num_faces());
    }
}

TEST(HalfEdge, HandleTopologyAndDirections) {
    for (const auto* name : {"cube.stl", "torus_simple.off", "square.off"}) {
        SCOPED_TRACE(name);
        auto        mesh = Mesh::from_file(config::meshes_directory() / name);
        const auto& m    = internal::cgal_mesh(mesh);
        for (Index i = 0; i < mesh.num_faces(); ++i) {
            SCOPED_TRACE(i);
            const FaceId              f{i};
            const auto                vs = vertices(mesh, f);
            const auto                ns = neighbours(mesh, f);
            std::array<HalfEdgeId, 3> hs{};
            for (std::size_t j = 0; j < 3; ++j) hs[j] = halfedge(mesh, vs[j], f);
            for (std::size_t j = 0; j < 3; ++j) {
                const auto h = hs[j];
                const auto t = twin(mesh, h);
                EXPECT_TRUE(h.is_valid());
                EXPECT_TRUE(t.is_valid());
                EXPECT_EQ(
                        internal::from_cgal(CGAL::source(internal::to_cgal(h), m)),
                        vs[j]
                );
                EXPECT_EQ(face(mesh, h), f);
                EXPECT_EQ(prev(mesh, h), hs[(j + 2) % 3]);
                EXPECT_EQ(prev(mesh, prev(mesh, prev(mesh, h))), h);
                EXPECT_EQ(face(mesh, prev(mesh, h)), f);
                EXPECT_EQ(twin(mesh, t), h);
                EXPECT_EQ(ns[j], face(mesh, twin(mesh, prev(mesh, h))));
                const Vec3d p = position(mesh, vs[j]);
                const Vec3d q = position(mesh, vs[(j + 1) % 3]);
                const Vec3d r = position(mesh, vs[(j + 2) % 3]);
                const Vec3d d = direction(mesh, h);
                EXPECT_NEAR((d - (q - p)).norm(), 0.0, 1e-12);
                EXPECT_NEAR((direction(mesh, t) + d).norm(), 0.0, 1e-12);
                const Vec3d inward = inbound_direction(mesh, h);
                EXPECT_NEAR(inward.norm(), 1.0, 1e-12);
                EXPECT_NEAR(inward.dot(d.normalized()), 0.0, 1e-12);
                EXPECT_NEAR(inward.dot(normal(mesh, f)), 0.0, 1e-12);
                EXPECT_GT(inward.dot(r - p), 0.0);
            }
            Index outside = 0;
            while (VertexId{outside} == vs[0] || VertexId{outside} == vs[1]
                   || VertexId{outside} == vs[2])
                ++outside;
            ASSERT_LT(outside, mesh.num_vertices());
            EXPECT_THROW(
                    static_cast<void>(halfedge(mesh, VertexId{outside}, f)),
                    std::runtime_error
            );
        }
        const auto  h     = halfedge(mesh, vertices(mesh, FaceId{0})[0], FaceId{0});
        const Vec3d d     = direction(mesh, h);
        auto        moved = std::move(mesh);
        EXPECT_EQ(face(moved, h), FaceId{0});
        EXPECT_NEAR((direction(moved, h) - d).norm(), 0.0, 1e-12);
    }
}

TEST(MeshHandle, ValueSemanticsAndCgalRoundTrip) {
    auto check = []<typename Id>() {
        using CgalId = decltype(internal::to_cgal(Id{}));
        EXPECT_FALSE(Id{}.is_valid());
        EXPECT_EQ(internal::from_cgal(CgalId{}), Id{});
        EXPECT_TRUE(Id{0}.is_valid());
        EXPECT_EQ(Id{17}, Id{17});
        EXPECT_LT(Id{3}, Id{17});
        EXPECT_EQ(static_cast<Index>(Id{17}), 17U);
        const std::unordered_set<Id> values{Id{}, Id{3}, Id{17}, Id{17}};
        EXPECT_EQ(values.size(), 3U);
        EXPECT_TRUE(values.contains(Id{17}));
        EXPECT_EQ(std::hash<Id>{}(Id{17}), std::hash<Index>{}(17));
        for (const Index index : {0U, 17U, invalid_index}) {
            EXPECT_EQ(internal::from_cgal(internal::to_cgal(Id{index})), Id{index});
        }
    };
    check.operator()<VertexId>();
    check.operator()<FaceId>();
    check.operator()<HalfEdgeId>();
}

TEST(Point, ValidationAndStructuralEquality) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    EXPECT_TRUE(Point{}.is_undefined());
    EXPECT_EQ(Point{}, Point(PointOnVertex{}));
    const auto bad_edge = [](HalfEdgeId h, double t) {
        return Point(PointOnEdge{.he = h, .t = t});
    };
    EXPECT_THROW(bad_edge(HalfEdgeId{}, 0.5), std::runtime_error);
    for (const double t :
         {0.0,
          1.0,
          -0.1,
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW(bad_edge(HalfEdgeId{0}, t), std::runtime_error);
    for (const Vec3d b : {
                 Vec3d{0,     0.5, 0.5},
                 Vec3d{-0.1,  0.5, 0.6},
                 Vec3d{0.2,   0.2, 0.2},
                 Vec3d{1e-14, 0.5, 0.5},
                 Vec3d{NAN,   0.5, 0.5}
    }) {
        EXPECT_THROW((Point(mesh, FaceId{0}, b, false)), std::runtime_error);
    }
    EXPECT_THROW((Point(mesh, FaceId{}, {0.2, 0.3, 0.5})), std::runtime_error);
    EXPECT_THROW((Point(mesh, FaceId{999999}, {0.2, 0.3, 0.5})), std::runtime_error);
    EXPECT_THROW(
            (Point(PointInFace{
                    FaceId{},
                    {0.2, 0.3, 0.5}
    })),
            std::runtime_error
    );
    const PointOnEdge e{.he = HalfEdgeId{0}, .t = 0.37};
    const Point       a{e};
    const Point       b{flip(mesh, e)};
    EXPECT_NE(a, b);
    EXPECT_TRUE(same_location(mesh, a, b));
    EXPECT_EQ(flip(mesh, flip(mesh, e)), e);
}

TEST(Point, CachedFaceLocationMatchesFallback) {
    using internal::CgalImpl;
    auto       mesh  = Mesh::from_file(config::meshes_directory() / "torus_simple.off");
    const auto check = [&mesh] {
        const auto& cgal = mesh.cgal();
        const auto& m    = cgal._mesh;
        for (const auto h : m.halfedges()) {
            // CGAL stores the two orientations of each edge consecutively.
            if (h.idx() >= 16) break;
            for (const double t : {0.13, 0.37, 0.5, 0.81}) {
                const Point point(PointOnEdge{.he = internal::from_cgal(h), .t = t});
                const auto  fallback = internal::face_location(m, point);
                const auto cached = internal::face_location(m, point, &cgal._aabb_tree);
                EXPECT_EQ(cached.first, fallback.first);
                EXPECT_EQ(cached.second, fallback.second);
                const auto cached_position =
                        CgalImpl::ShortestPath::point(cached.first, cached.second, m);
                EXPECT_EQ(
                        cached_position,
                        CgalImpl::ShortestPath::point(
                                fallback.first, fallback.second, m
                        )
                );
                EXPECT_NEAR(
                        (internal::convert(cached_position) - position(mesh, point))
                                .norm(),
                        0.0,
                        1e-12
                );
            }
        }
        const Point invalid(PointOnEdge{.he = HalfEdgeId{999999}, .t = 0.37});
        EXPECT_THROW(internal::face_location(m, invalid), std::runtime_error);
        EXPECT_THROW(
                internal::face_location(m, invalid, &cgal._aabb_tree),
                std::runtime_error
        );
    };
    check();
    mesh.scale(2.7);
    check();
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.rotate(Eigen::AngleAxisd(0.43, Vec3d{1, 2, 3}.normalized()));
    transform.translate(Vec3d{0.31, -0.27, 1.19});
    mesh.transform(transform);
    check();
}

TEST(Point, BarycentricTopologyAndMoveInvariants) {
    auto        mesh = Mesh::from_file(config::meshes_directory() / "torus_simple.off");
    const auto& m    = internal::cgal_mesh(mesh);
    for (Index i = 0; i < mesh.num_faces(); ++i) {
        const FaceId     f{i};
        const auto       h = CGAL::halfedge(internal::to_cgal(f), m);
        const std::array vs{
                CGAL::source(h, m),
                CGAL::target(h, m),
                CGAL::target(CGAL::next(h, m), m)
        };
        for (int j = 0; j < 3; ++j) {
            Vec3d b = Vec3d::Zero();
            b[j]    = 1;
            const Point v(mesh, f, b);
            ASSERT_NE(v.get_as<PointOnVertex>(), nullptr);
            EXPECT_EQ(
                    v.get_as<PointOnVertex>()->v,
                    internal::from_cgal(vs[static_cast<std::size_t>(j)])
            );
            b[(j + 1) % 3] = 0.37;
            b[j]           = 0.63;
            const Point e(mesh, f, b);
            ASSERT_NE(e.get_as<PointOnEdge>(), nullptr);
            const auto expected = internal::convert(
                    internal::CgalImpl::ShortestPath::point(
                            internal::to_cgal(f), {b[0], b[1], b[2]}, m
                    )
            );
            EXPECT_NEAR((position(mesh, e) - expected).norm(), 0.0, 1e-12);
            const auto loc = internal::face_location(m, e, f);
            EXPECT_EQ(loc.first, internal::to_cgal(f));
            EXPECT_NEAR(
                    (internal::convert(
                             internal::CgalImpl::ShortestPath::point(
                                     loc.first, loc.second, m
                             )
                     )
                     - expected)
                            .norm(),
                    0.0,
                    1e-12
            );
        }
    }
    const std::array<Point, 3> points{
            PointOnVertex{VertexId{0}},
            PointOnEdge{.he = HalfEdgeId{0}, .t = 0.37},
            Point(mesh, FaceId{0},
            {0.2, 0.3, 0.5}
            )
    };
    std::array<CartesianPoint, 3> positions{};
    for (std::size_t i = 0; i < points.size(); ++i)
        positions[i] = position(mesh, points[i]);
    auto moved    = std::move(mesh);
    auto assigned = Mesh::from_file(config::meshes_directory() / "square.off");
    assigned      = std::move(moved);
    for (std::size_t i = 0; i < points.size(); ++i) {
        EXPECT_NEAR((position(assigned, points[i]) - positions[i]).norm(), 0.0, 1e-12);
        EXPECT_TRUE(same_location(
                assigned, points[i], from_cartesian(assigned, positions[i])
        ));
    }
}

}  // namespace mdv::mesh
