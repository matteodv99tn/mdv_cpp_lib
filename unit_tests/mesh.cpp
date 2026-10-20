#include "mdv/mesh/mesh.hpp"

#include <algorithm>
#include <gtest/gtest.h>
#include <numbers>
#include <random>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"

namespace mdv::mesh {
namespace fs = std::filesystem;

TEST(MeshGeodesic, RandomPoints) {
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);

    for (std::size_t i = 0; i < 40; ++i) {
        const auto p0 = Point::random(mesh);
        const auto p1 = Point::random(mesh);

        const auto g0 = mesh.build_geodesic(p0, p1);
        const auto g1 = mesh.build_geodesic(p1, p0);
        EXPECT_NEAR(length(g0), length(g1), 1e-9);
    }
}

TEST(MeshGeodesic, RandomVertexPoints) {
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);

    std::random_device                   dev;
    std::mt19937                         rng(dev());
    std::uniform_int_distribution<Index> dist(
            0, static_cast<Index>(mesh.num_vertices() - 1)
    );


    for (std::size_t i = 0; i < 40; ++i) {
        const Vertex& v0 = mesh.vertex(dist(rng));
        const Vertex& v1 = mesh.vertex(dist(rng));
        const auto p0 = Point::from_cartesian(mesh, position(mesh, VertexId{v0.id()}));
        const auto p1 = Point::from_cartesian(mesh, position(mesh, VertexId{v1.id()}));

        const auto g0 = mesh.build_geodesic(p0, p1);
        const auto g1 = mesh.build_geodesic(p1, p0);
        EXPECT_NEAR(length(g0), length(g1), 1e-9)
                << "v0: " << describe(mesh, VertexId{v0.id()})
                << "\nv1: " << describe(mesh, VertexId{v1.id()});
    }
}

TEST(MeshGeodesic, RandomEdgePoints) {
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);

    std::random_device                   dev;
    std::mt19937                         rng(dev());
    std::uniform_int_distribution<Index> dist(
            0, static_cast<Index>(mesh.num_vertices() - 1)
    );


    using EdgeDescriptor = Point::PointOnEdgeDescriptor;
    for (std::size_t i = 0; i < 40; ++i) {
        const auto p0 = Point(EdgeDescriptor::random(mesh));
        const auto p1 = Point(EdgeDescriptor::random(mesh));

        const auto g0 = mesh.build_geodesic(p0, p1);
        const auto g1 = mesh.build_geodesic(p1, p0);

        ASSERT_NEAR(length(g0), length(g1), 1e-9)
                << "Iteration " << i << "\np0: " << p0.describe()
                << "\np1: " << p1.describe();
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
    const auto face = mesh.face(0);
    const auto edge = Point::PointOnEdgeDescriptor(
            HalfEdge{
                    mesh,
                    halfedge(
                            mesh,
                            vertices(mesh, FaceId{face.id()})[0],
                            FaceId{face.id()}
                    )
                            .value
            },
            0.3
    );
    const std::vector<Point> points{
            Point(mesh.vertex(0)),
            Point(mesh.vertex(17)),
            Point(edge),
            Point(edge.display_in_opposite_halfedge()),
            Point(Point::PointInFaceDescriptor(mesh.face(0), {0.2, 0.3, 0.5})),
            Point(Point::PointInFaceDescriptor(mesh.face(23), {0.4, 0.35, 0.25}))
    };
    for (std::size_t i = 0; i < points.size(); ++i) {
        SCOPED_TRACE(i);
        const auto& from      = points[i];
        const auto  projected = Point::from_cartesian(mesh, from.position());
        EXPECT_NEAR((projected.position() - from.position()).norm(), 0.0, 1e-9);
        EXPECT_NEAR(length(mesh.build_geodesic(from, from)), 0.0, 1e-9);
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            SCOPED_TRACE(j);
            const auto& to      = points[j];
            const auto  forward = mesh.build_geodesic(from, to);
            const auto  reverse = mesh.build_geodesic(to, from);
            if ((to.position() - from.position()).norm() < 1e-12) {
                // Coincident positions (including opposite edge descriptors) have
                // an empty polyline in the current API.
                EXPECT_NEAR(length(forward), 0.0, 1e-9);
                EXPECT_NEAR(length(reverse), 0.0, 1e-9);
                continue;
            }
            ASSERT_FALSE(forward.empty());
            ASSERT_FALSE(reverse.empty());
            EXPECT_NEAR((forward.front() - from.position()).norm(), 0.0, 1e-9);
            EXPECT_NEAR((forward.back() - to.position()).norm(), 0.0, 1e-9);
            EXPECT_NEAR((reverse.front() - to.position()).norm(), 0.0, 1e-9);
            EXPECT_NEAR((reverse.back() - from.position()).norm(), 0.0, 1e-9);
            EXPECT_NEAR(length(forward), length(reverse), 1e-9);
            EXPECT_GE(length(forward) + 1e-9, (to.position() - from.position()).norm());
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
    const auto first = mesh.face(0);
    const auto copy  = mesh.face(0);
    EXPECT_EQ(first, copy);
    EXPECT_EQ(mesh.face(0), mesh.face(0));
    EXPECT_NE(mesh.face(0), mesh.face(1));
    EXPECT_TRUE(Point{}.is_undefined());
    EXPECT_FALSE(Point{mesh.vertex(0)}.is_undefined());
    const Point interior{
            Point::PointInFaceDescriptor{first, {0.2, 0.3, 0.5}}
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
        EXPECT_LT(random_face(mesh).value, mesh.num_faces());
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

}  // namespace mdv::mesh
