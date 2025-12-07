#include "mdv/mesh/kernel.hpp"

#include <chrono>
#include <gtest/gtest.h>

#include "mdv/config.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"

using namespace mdv::mesh;
using namespace mdv::condition;
using mdv::Vec3d;
using std::filesystem::path;

TEST(MeshKernel, Square_SinglePointSet) {
    const path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto mesh      = Mesh::from_file(mesh_path);

    auto to_ms = [](const auto& delta) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(delta).count();
    };

    const long         n_pts = 20;
    std::vector<Point> points;
    points.reserve(n_pts);
    for (long i = 0; i < n_pts; ++i)
        points.emplace_back(Point::PointOnEdgeDescriptor::random(mesh));

    Eigen::MatrixXd mesh_d(n_pts, n_pts);
    auto            start = std::chrono::high_resolution_clock::now();
    for (long i = 0; i < n_pts; ++i) {
        for (long j = 0; j < n_pts; ++j) {
            mesh_d(i, j) = length(mesh.build_geodesic(points[i], points[j]));
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    std::cout << "Naive matrix computation time: " << to_ms(stop - start) << "ms\n";

    MeshKernel kernel(mesh);
    start                          = std::chrono::high_resolution_clock::now();
    const Eigen::MatrixXd kernel_d = kernel.distance_matrix(points, points);
    stop                           = std::chrono::high_resolution_clock::now();
    std::cout << "MeshKernel matrix computation time: " << to_ms(stop - start)
              << "ms\n";

    for (long i = 0; i < n_pts; ++i) {
        for (long j = 0; j < n_pts; ++j) {
            EXPECT_NEAR(kernel_d(i, j), mesh_d(i, j), 1e-9)
                    << "When i=" << i << ", j=" << j;
        }
    }
}

TEST(MeshKernel, Square_DoublePointSet) {
    const path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto mesh      = Mesh::from_file(mesh_path);

    const long         n_pts_a = 20;
    const long         n_pts_b = 10;
    std::vector<Point> points_a, points_b;
    points_a.reserve(n_pts_a);
    points_b.reserve(n_pts_b);
    for (long i = 0; i < n_pts_a; ++i)
        points_a.emplace_back(Point::PointOnEdgeDescriptor::random(mesh));
    for (long i = 0; i < n_pts_b; ++i)
        points_b.emplace_back(Point::PointOnEdgeDescriptor::random(mesh));

    Eigen::MatrixXd mesh_d(n_pts_a, n_pts_b);
    for (long i = 0; i < n_pts_a; ++i) {
        for (long j = 0; j < n_pts_b; ++j) {
            mesh_d(i, j) = length(mesh.build_geodesic(points_a[i], points_b[j]));
        }
    }

    MeshKernel            kernel(mesh);
    const Eigen::MatrixXd kernel_d = kernel.distance_matrix(points_a, points_b);

    for (long i = 0; i < n_pts_a; ++i) {
        for (long j = 0; j < n_pts_b; ++j) {
            EXPECT_NEAR(kernel_d(i, j), mesh_d(i, j), 1e-9)
                    << "When i=" << i << ", j=" << j;
        }
    }
}
