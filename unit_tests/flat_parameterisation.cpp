#include "mdv/mesh/flat_parameterisation.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <gtest/gtest.h>

#include "mdv/config.hpp"
#include "mdv/mesh/flat_parameterisation.hpp"
#include "mdv/mesh/mesh.hpp"

using namespace mdv::mesh;

using std::filesystem::path;

TEST(MdvMesh, FlatParameterisation) {
    const path mesh_path = mdv::config::meshes_directory() / "bunny.off";
    const auto full_mesh = Mesh::from_file(mesh_path);
    auto       mesh =
            Mesh::extract_normal_bounded_surface(full_mesh, full_mesh.vertex(0), 90.0);
    FlatParameterisation map(mesh);

    ASSERT_TRUE(map.is_one_to_one_mapping());

    ASSERT_FALSE(map.is_inside_mesh(map.max_uv() + Eigen::Vector2d{0.1, 0.1}));
    ASSERT_FALSE(map.is_inside_mesh(map.min_uv() - Eigen::Vector2d{0.1, 0.1}));

    // Check bijective mapping of vertices
    for (long i = 0; i < mesh.num_vertices(); ++i) {
        const auto   v   = mesh.vertex(i);
        const auto   uv  = map.project(v);
        const auto   rec = map.retrieve(uv);
        const double e   = (v.position() - rec.position()).norm();
        ASSERT_TRUE(map.is_inside_mesh(uv));
        ASSERT_LT(e, 1e-12);
    }

    // Check bijective mapping of points centered in the faces
    constexpr double b = 1.0 / 3.0;
    for (long i = 0; i < mesh.num_faces(); ++i) {
        const auto   pt  = Point::PointInFaceDescriptor(mesh.face(i), {b, b, b});
        const auto   uv  = map.project(pt);
        const auto   rec = map.retrieve(uv);
        const double e   = (pt.position() - rec.position()).norm();
        ASSERT_TRUE(map.is_inside_mesh(uv));
        ASSERT_LT(e, 1e-12);
    }
}
