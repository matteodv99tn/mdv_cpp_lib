#include <gtest/gtest.h>
#include <variant>

#include "mdv/config.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/mesh.hpp"

using namespace mdv::mesh;
using namespace mdv::condition;
using mdv::Vec3d;
using std::filesystem::path;

TEST(Mesh, Square) {
    const path mesh_path = mdv::config::meshes_directory() / "square.off";
    const auto mesh      = Mesh::from_file(mesh_path);

    EXPECT_EQ(mesh.num_faces(), 2);
    EXPECT_EQ(mesh.num_vertices(), 4);

    EXPECT_TRUE(are_equal(Vec3d{0.0, 0.0, 0.0}, mesh.vertex(0).position()));
    EXPECT_TRUE(are_equal(Vec3d{1.0, 0.0, 0.0}, mesh.vertex(1).position()));
    EXPECT_TRUE(are_equal(Vec3d{1.0, 1.0, 0.0}, mesh.vertex(2).position()));
    EXPECT_TRUE(are_equal(Vec3d{0.0, 1.0, 0.0}, mesh.vertex(3).position()));

    const auto pt_on_v0 = Point::from_cartesian(mesh, Vec3d{0.0, 0.0, 0.0});
    EXPECT_NO_THROW(std::get<Point::PointOnVertexDescriptor>(pt_on_v0.descriptor()));
    EXPECT_THROW(
            std::get<Point::PointOnEdgeDescriptor>(pt_on_v0.descriptor()),
            std::bad_variant_access
    );
    EXPECT_THROW(
            std::get<Point::PointInFaceDescriptor>(pt_on_v0.descriptor()),
            std::bad_variant_access
    );
    EXPECT_TRUE(are_equal(pt_on_v0.position(), Vec3d{0.0, 0.0, 0.0}));

    const auto pt_on_edge = Point::from_cartesian(mesh, Vec3d{0.5, 0.0, 0.0});
    EXPECT_THROW(
            std::get<Point::PointOnVertexDescriptor>(pt_on_edge.descriptor()),
            std::bad_variant_access
    );
    EXPECT_NO_THROW(std::get<Point::PointOnEdgeDescriptor>(pt_on_edge.descriptor()));
    EXPECT_THROW(
            std::get<Point::PointInFaceDescriptor>(pt_on_edge.descriptor()),
            std::bad_variant_access
    );
    EXPECT_TRUE(are_equal(pt_on_edge.position(), Vec3d{0.5, 0.0, 0.0}));

    auto opposite_description =
            std::get<Point::PointOnEdgeDescriptor>(pt_on_edge.descriptor())
                    .display_in_opposite_halfedge();
    EXPECT_TRUE(are_equal(opposite_description.position(), Vec3d{0.5, 0.0, 0.0}));

    const auto pt_on_diag_edge = Point::from_cartesian(mesh, Vec3d{0.8, 0.2, 0.0});
    EXPECT_THROW(
            std::get<Point::PointOnVertexDescriptor>(pt_on_diag_edge.descriptor()),
            std::bad_variant_access
    );
    EXPECT_NO_THROW(
            std::get<Point::PointOnEdgeDescriptor>(pt_on_diag_edge.descriptor())
    );
    EXPECT_THROW(
            std::get<Point::PointInFaceDescriptor>(pt_on_diag_edge.descriptor()),
            std::bad_variant_access
    );
    EXPECT_TRUE(are_equal(pt_on_diag_edge.position(), Vec3d{0.8, 0.2, 0.0}));
    opposite_description =
            std::get<Point::PointOnEdgeDescriptor>(pt_on_diag_edge.descriptor())
                    .display_in_opposite_halfedge();
    EXPECT_TRUE(are_equal(opposite_description.position(), Vec3d{0.8, 0.2, 0.0}));

    const auto pt_in_face = Point::from_cartesian(mesh, Vec3d{0.2, 0.2, 0.0});
    EXPECT_THROW(
            std::get<Point::PointOnVertexDescriptor>(pt_in_face.descriptor()),
            std::bad_variant_access
    );
    EXPECT_THROW(
            std::get<Point::PointOnEdgeDescriptor>(pt_in_face.descriptor()),
            std::bad_variant_access
    );
    EXPECT_NO_THROW(std::get<Point::PointInFaceDescriptor>(pt_in_face.descriptor()));
    EXPECT_TRUE(are_equal(pt_in_face.position(), Vec3d{0.2, 0.2, 0.0}));
}
