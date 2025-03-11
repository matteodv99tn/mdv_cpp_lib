#include "mdv/utils/conditions.hpp"

#include <gtest/gtest.h>

#include "mdv/utils/concepts.hpp"

TEST(MdvCondition, Parallel3dCheck) {
    using mdv::condition::are_parallel;
    const Eigen::Vector3d v1({0.0, 0.0, 1.0});
    const Eigen::Vector3d v2({0.0, 0.0, -1.0});
    const Eigen::Vector3d v3({0.0, 0.0, 0.0});
    const Eigen::Vector3d v4({1.0, 0.0, 0.0});

    EXPECT_TRUE(are_parallel(v1, v1));
    EXPECT_TRUE(are_parallel(v1, v2));
    EXPECT_FALSE(are_parallel(v1, v3));
    EXPECT_FALSE(are_parallel(v1, v4));
}

TEST(MdvCondition, Parallel3dExpressionCheck) {
    using mdv::condition::are_parallel;
    const Eigen::Vector3d v1({0.0, 1.0, 1.0});
    const Eigen::Vector3d v2({1.0, 0.0, 0.0});
    const Eigen::Vector3d v3({0.0, 0.0, 0.0});

    EXPECT_TRUE(are_parallel(v1 + v3, v1 - v3));
    EXPECT_TRUE(are_parallel(v1, -v1));
    EXPECT_FALSE(are_parallel(v1, v1 + v2));
}

TEST(MdvCondition, ParallelGenericVector) {
    using mdv::condition::are_parallel;

    const std::vector<long> vec_dimensions = {3, 10, 100, 1000};

    for (const long dim : vec_dimensions) {
        Eigen::VectorXd v1(dim);
        Eigen::VectorXd v2(dim);
        Eigen::VectorXd v3(dim);
        Eigen::VectorXd v4(dim);
        v1 = Eigen::VectorXd::Random(dim);
        v2 = -v1;
        v3 = v1;
        v3(0) += 1.0;  // Make it non-parallel
        v4 = Eigen::VectorXd::Zero(dim);

        EXPECT_TRUE(are_parallel(v1, v1));
        EXPECT_TRUE(are_parallel(v1, v2));
        EXPECT_FALSE(are_parallel(v1, v3));
        EXPECT_FALSE(are_parallel(v1, v4));
    }
}

TEST(MdvCondition, Orthogonal3dVectors) {
    using mdv::condition::are_orthogonal;
    const Eigen::Vector3d v1({0.0, 0.0, 1.0});
    const Eigen::Vector3d v2({0.0, 0.0, -1.0});
    const Eigen::Vector3d v3({0.0, 0.0, 0.0});
    const Eigen::Vector3d v4({1.0, 0.0, 0.0});
    const Eigen::Vector3d v5({-1.0, 0.0, 0.0});

    EXPECT_FALSE(are_orthogonal(v1, v1));
    EXPECT_FALSE(are_orthogonal(v1, v2));
    EXPECT_TRUE(are_orthogonal(v1, v3));
    EXPECT_TRUE(are_orthogonal(v1, v4));
    EXPECT_TRUE(are_orthogonal(v1, v5));
}

// Benchmarks
// TEST_CASE("Eigen conditional benchmarks", "[conditions][benchmark]") {
//     using mdv::condition::are_orthogonal;
//     using mdv::condition::are_parallel;
//
//     const std::vector<long> vec_dimensions = {3, 10, 100, 1000, 10000};
//
//     for (long dim : vec_dimensions) {
//         BENCHMARK_ADVANCED("Parallel check - vector size: " + std::to_string(dim))
//         (Catch::Benchmark::Chronometer meter) {
//             Eigen::VectorXd v1(dim);
//             Eigen::VectorXd v2(dim);
//             v2 = v1;
//             v2(0) += 1.0;
//
//             meter.measure([&v1, &v2] { return are_parallel(v1, v2); });
//         };
//         BENCHMARK_ADVANCED("Orthogonality check")
//         (Catch::Benchmark::Chronometer meter) {
//             Eigen::VectorXd v1(dim);
//             Eigen::VectorXd v2(dim);
//             v2 = v1;
//             v2(0) += 1.0;
//
//             meter.measure([&v1, &v2] { return are_orthogonal(v1, v2); });
//         };
//     }
// }
//
