#include <catch2/catch_test_macros.hpp>
#include <mdv/utils/concepts.hpp>
#include <mdv/utils/conditions.hpp>

TEST_CASE("Parallel condition check - 3d vector", "[conditions]") {
    using mdv::condition::are_parallel;
    const Eigen::Vector3d v1({0.0, 0.0, 1.0});
    const Eigen::Vector3d v2({0.0, 0.0, -1.0});
    const Eigen::Vector3d v3({0.0, 0.0, 0.0});
    const Eigen::Vector3d v4({1.0, 0.0, 0.0});

    REQUIRE(are_parallel(v1, v1));
    REQUIRE(are_parallel(v1, v2));
    REQUIRE(!are_parallel(v1, v3));
    REQUIRE(!are_parallel(v1, v4));
}

TEST_CASE("Parallel condition check - 3d vector like expression", "[conditions]") {
    using mdv::condition::are_parallel;
    const Eigen::Vector3d v1({0.0, 1.0, 1.0});
    const Eigen::Vector3d v2({1.0, 0.0, 0.0});
    const Eigen::Vector3d v3({0.0, 0.0, 0.0}); 

    REQUIRE(are_parallel(v1 + v3, v1 - v3));
    REQUIRE(are_parallel(v1, -v1));
    REQUIRE(!are_parallel(v1, v1 + v2));
}

TEST_CASE("Parallel condition check - generic size vector", "[conditions]") {
    using mdv::condition::are_parallel;
    const long      dim = 10;
    Eigen::VectorXd v1(dim);
    Eigen::VectorXd v2(dim);
    Eigen::VectorXd v3(dim);
    Eigen::VectorXd v4(dim);

    v1 = Eigen::VectorXd::Random(dim);
    v2 = -v1;
    v3 = v1;
    v3(0) += 1.0;  // Make it non-parallel
    v4 = Eigen::VectorXd::Zero(dim);


    REQUIRE(are_parallel(v1, v1));
    REQUIRE(are_parallel(v1, v2));
    REQUIRE(!are_parallel(v1, v3));
    REQUIRE(!are_parallel(v1, v4));
}

TEST_CASE("Orthogonality condition check - 3d vector", "[conditions]") {
    using mdv::condition::are_orthogonal;
    const Eigen::Vector3d v1({0.0, 0.0, 1.0});
    const Eigen::Vector3d v2({0.0, 0.0, -1.0});
    const Eigen::Vector3d v3({0.0, 0.0, 0.0});
    const Eigen::Vector3d v4({1.0, 0.0, 0.0});
    const Eigen::Vector3d v5({-1.0, 0.0, 0.0});

    REQUIRE(!are_orthogonal(v1, v1));
    REQUIRE(!are_orthogonal(v1, v2));
    REQUIRE(!are_orthogonal(v1, v3));
    REQUIRE(are_orthogonal(v1, v4));
    REQUIRE(are_orthogonal(v1, v5));
}
