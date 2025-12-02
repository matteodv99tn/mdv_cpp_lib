#include "mdv/mesh/mesh.hpp"

#include <gtest/gtest.h>
#include <random>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"


using namespace mdv::mesh;
using std::filesystem::path;

TEST(MeshGeodesic, RandomPoints) {
    const path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto mesh      = Mesh::from_file(mesh_path);

    for (std::size_t i = 0; i < 40; ++i) {
        const auto p0 = Point::random(mesh);
        const auto p1 = Point::random(mesh);

        const auto g0 = mesh.build_geodesic(p0, p1);
        const auto g1 = mesh.build_geodesic(p1, p0);
        EXPECT_NEAR(length(g0), length(g1), 1e-9);
    }
}

TEST(MeshGeodesic, RandomVertexPoints) {
    const path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto mesh      = Mesh::from_file(mesh_path);

    std::random_device                  dev;
    std::mt19937                        rng(dev());
    std::uniform_int_distribution<long> dist(0, mesh.num_vertices() - 1);


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
    const path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto mesh      = Mesh::from_file(mesh_path);

    std::random_device                  dev;
    std::mt19937                        rng(dev());
    std::uniform_int_distribution<long> dist(0, mesh.num_vertices() - 1);


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
