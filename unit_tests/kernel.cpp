#include "mdv/mesh/kernel.hpp"

#include <chrono>
#include <gtest/gtest.h>

#include "mdv/config.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/sampler.hpp"

namespace mdv::mesh {
namespace fs = std::filesystem;

TEST(MeshKernel, Square_SinglePointSet) {
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);

    auto to_ms = [](const auto& delta) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(delta).count();
    };

    const long n_pts = 20;
    Sampler    sampler(mesh, 42);
    const auto points = sampler.points_on_edges(n_pts);

    Eigen::MatrixXd mesh_d(n_pts, n_pts);
    auto            start = std::chrono::high_resolution_clock::now();
    for (long i = 0; i < n_pts; ++i) {
        for (long j = 0; j < n_pts; ++j) {
            mesh_d(i, j) = length(build_geodesic(
                    mesh,
                    points[static_cast<std::size_t>(i)],
                    points[static_cast<std::size_t>(j)]
            ));
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
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);

    const long n_pts_a = 20;
    const long n_pts_b = 10;
    Sampler    sampler(mesh, 42);
    const auto points_a = sampler.points_on_edges(n_pts_a);
    const auto points_b = sampler.points_on_edges(n_pts_b);

    Eigen::MatrixXd mesh_d(n_pts_a, n_pts_b);
    for (long i = 0; i < n_pts_a; ++i) {
        for (long j = 0; j < n_pts_b; ++j) {
            mesh_d(i, j) = length(build_geodesic(
                    mesh,
                    points_a[static_cast<std::size_t>(i)],
                    points_b[static_cast<std::size_t>(j)]
            ));
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

TEST(MeshKernel, DeterministicDistanceMatrixInvariants) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "torus_simple.off");
    const FaceId      f{0};
    const PointOnEdge edge{.he = halfedge(mesh, vertices(mesh, f)[0], f), .t = 0.3};
    const std::vector<Point> points{
            Point(PointOnVertex{VertexId{0}}),
            Point(edge),
            Point(mesh, FaceId{23}, {0.2, 0.3, 0.5}),
            Point(flip(mesh, edge))
    };
    const std::vector<Point> other{
            points[2], points[0], Point(mesh, FaceId{7}, {0.4, 0.35, 0.25})
    };
    MeshKernel kernel(mesh);
    const auto square  = kernel.distance_matrix(points, points);
    const auto cross   = kernel.distance_matrix(points, other);
    const auto reverse = kernel.distance_matrix(other, points);
    ASSERT_EQ(square.rows(), static_cast<Eigen::Index>(points.size()));
    ASSERT_EQ(square.cols(), square.rows());
    ASSERT_EQ(cross.rows(), square.rows());
    ASSERT_EQ(cross.cols(), static_cast<Eigen::Index>(other.size()));
    ASSERT_EQ(reverse.rows(), cross.cols());
    ASSERT_EQ(reverse.cols(), cross.rows());
    for (Eigen::Index i = 0; i < square.rows(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_NEAR(square(i, i), 0.0, 1e-9);
        for (Eigen::Index j = 0; j < square.cols(); ++j) {
            SCOPED_TRACE(j);
            EXPECT_NEAR(square(i, j), square(j, i), 1e-9);
            EXPECT_NEAR(
                    square(i, j),
                    length(build_geodesic(
                            mesh,
                            points[static_cast<std::size_t>(i)],
                            points[static_cast<std::size_t>(j)]
                    )),
                    1e-9
            );
        }
        for (Eigen::Index j = 0; j < cross.cols(); ++j) {
            SCOPED_TRACE(j);
            EXPECT_NEAR(cross(i, j), reverse(j, i), 1e-9);
            EXPECT_NEAR(
                    cross(i, j),
                    length(build_geodesic(
                            mesh,
                            points[static_cast<std::size_t>(i)],
                            other[static_cast<std::size_t>(j)]
                    )),
                    1e-9
            );
        }
        // Opposite edge descriptors denote the same position and distances.
        EXPECT_NEAR(square(i, 1), square(i, 3), 1e-9);
    }
}

}  // namespace mdv::mesh
