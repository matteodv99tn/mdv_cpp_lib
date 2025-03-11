#include "mdv/mesh/tangent_vector.hpp"

#include <gtest/gtest.h>

#include "mdv/config.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"

using mdv::mesh::Mesh;
using mdv::mesh::TangentVector;

TEST(MdvMesh, TangentVectorUnitRandomInitialisation) {
    const std::string mesh_path =
            std::string(mdv::config::mesh_dir) + "/torus_simple.off";
    const auto mesh = Mesh::from_file(mesh_path);

    for (int i{0}; i < 256; ++i) {
        const auto pt  = Mesh::Point::random(mesh);
        const auto vec = TangentVector::unit_random(pt);
        ASSERT_TRUE(mdv::condition::is_unit_norm(vec.tip() - pt.position()));
    }
}

TEST(MdvMesh, TangentVectorFromTipInitialisation) {
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;
    const std::string mesh_path =
            std::string(mdv::config::mesh_dir) + "/torus_simple.off";
    const auto mesh     = Mesh::from_file(mesh_path);
    const auto pref_dir = Vec3::UnitX();

    for (int i{0}; i < 256; ++i) {
        const auto pt    = Mesh::Point::random(mesh);
        const auto n     = pt.face().normal();
        const Vec3 delta = (Mat3::Identity() - n * n.transpose()) * pref_dir;
        const Vec3 tip   = pt.position() + delta;

        const auto vec = TangentVector::from_tip_position(pt, tip);
        ASSERT_TRUE(mdv::condition::are_orthogonal(n, delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.cartesian_vector() - delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.tip() - tip));
    }
}

TEST(MdvMesh, TangentVectorDifference) {
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;
    const std::string mesh_path =
            std::string(mdv::config::mesh_dir) + "/torus_simple.off";
    const auto mesh     = Mesh::from_file(mesh_path);
    const auto pref_dir = Vec3::UnitX();

    for (int i{0}; i < 256; ++i) {
        const auto pt = Mesh::Point::random(mesh);
        const auto v1 = TangentVector::unit_random(pt);
        const auto v2 = TangentVector::unit_random(pt);

        const auto n     = pt.face().normal();
        const Vec3 delta = (Mat3::Identity() - n * n.transpose()) * pref_dir;
        const Vec3 tip   = pt.position() + delta;

        const auto vec = TangentVector::from_tip_position(pt, tip);
        ASSERT_TRUE(mdv::condition::are_orthogonal(n, delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.cartesian_vector() - delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.tip() - tip));
    }
}
