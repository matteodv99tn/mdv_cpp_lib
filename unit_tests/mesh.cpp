#include "mdv/mesh/mesh.hpp"

#include <gtest/gtest.h>
#include <numbers>
#include <random>
#include <vector>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"

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
        const auto    p0 = Point::from_cartesian(mesh, v0.position());
        const auto    p1 = Point::from_cartesian(mesh, v1.position());

        const auto g0 = mesh.build_geodesic(p0, p1);
        const auto g1 = mesh.build_geodesic(p1, p0);
        EXPECT_NEAR(length(g0), length(g1), 1e-9)
                << "v0: " << v0.describe() << "\nv1: " << v1.describe();
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
                static_cast<float>(mesh.vertex(i).total_curvature()),
                static_cast<float>(1.5 * M_PI)
        );
        ASSERT_FLOAT_EQ(
                static_cast<float>(mesh.vertex(i).gauss_curvature()),
                static_cast<float>(0.5 * M_PI)
        );
    }
}

TEST(MeshGeodesic, DeterministicDescriptorInvariants) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "torus_simple.off");
    const auto face = mesh.face(0);
    const auto edge = Point::PointOnEdgeDescriptor(
            HalfEdge{mesh.vertex(face.vertices_ids()[0]), face}, 0.3
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
            total_angle += mesh.vertex(i).total_curvature();
            curvature += mesh.vertex(i).gauss_curvature();
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

}  // namespace mdv::mesh
