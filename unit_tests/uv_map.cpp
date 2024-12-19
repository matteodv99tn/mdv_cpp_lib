#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <mdv/config.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/uv_map.hpp>
#include <mdv/utils/conditions.hpp>

TEST_CASE("Forward and inverse mappings", "[mesh][uv_map]") {
    using UvMap   = mdv::mesh::UvMap;
    using Point3d = mdv::mesh::Point3d;
    using UvCoord = UvMap::Domain;

    using mdv::condition::is_zero_norm;

    const Point3d v1{1.0, 1.0, 1.0};
    const Point3d v2{2.0, 1.0, 1.0};
    const Point3d v3{1.0, 2.0, 1.0};
    const UvMap   map(v1, v2, v3);

    {
        const UvCoord uv{0.0, 0.0};
        const Point3d p_des = v1;
        REQUIRE(is_zero_norm(p_des - map.forward_map(uv)));
        REQUIRE(is_zero_norm(uv - map.inverse_map(p_des)));
    }
    {
        const UvCoord uv{1.0, 0.0};
        const Point3d p_des = v2;
        REQUIRE(is_zero_norm(p_des - map.forward_map(uv)));
        REQUIRE(is_zero_norm(uv - map.inverse_map(p_des)));
    }
    {
        const UvCoord uv{0.0, 1.0};
        const Point3d p_des = v3;
        REQUIRE(is_zero_norm(p_des - map.forward_map(uv)));
        REQUIRE(is_zero_norm(uv - map.inverse_map(p_des)));
    }
    {
        const UvCoord uv{1.0, 1.0};
        const Point3d p_des{2.0, 2.0, 1.0};
        REQUIRE(is_zero_norm(p_des - map.forward_map(uv)));
        REQUIRE(is_zero_norm(uv - map.inverse_map(p_des)));
    }
}

TEST_CASE("Inverse mapping benchmarks", "[mesh][uv_map]") {
    Eigen::Matrix<double, 3, 2> transform;
    Eigen::Vector<double, 3>    pt{1.0, 1.0, 0.0};
    transform.col(0) = Eigen::Vector3d({1.0, 0.0, 0.0});
    transform.col(1) = Eigen::Vector3d({0.0, 1.0, 0.0});

    // Can't run: "transform" is not invertible
    // BENCHMARK("partialPivLu()") {
    //     return transform.partialPivLu().solve(pt);
    // };
    BENCHMARK("fullPivLu()") {
        return transform.fullPivLu().solve(pt);
    };
    BENCHMARK("householderQr()") {
        return transform.householderQr().solve(pt);
    };
    BENCHMARK("colPivHouseholderQr()") {
        return transform.colPivHouseholderQr().solve(pt);
    };
    BENCHMARK("fullPivHouseholderQr()") {
        return transform.fullPivHouseholderQr().solve(pt);
    };
    BENCHMARK("jacobiSvd()") {
        return transform.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(pt);
    };
    BENCHMARK("bdcSvd()") {
        return transform.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(pt);
    };
}
