#include "mdv/mesh/sampler.hpp"

#include <gtest/gtest.h>

#include "mdv/config.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"

namespace mdv::mesh {

TEST(Sampler, ReproducibleSequences) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "torus_simple.off");
    Sampler    a(mesh, 42);
    Sampler    b(mesh, 42);
    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(a.face(), b.face());
        EXPECT_EQ(a.point_on_vertex(), b.point_on_vertex());
        EXPECT_EQ(a.point_on_edge(), b.point_on_edge());
        const Point p = a.point_in_face();
        EXPECT_EQ(p, Point(b.point_in_face()));
        EXPECT_EQ(
                a.unit_tangent(p).cartesian_vector(),
                b.unit_tangent(p).cartesian_vector()
        );
    }
    EXPECT_NE(
            Sampler(mesh, 42).points_in_faces(100),
            Sampler(mesh, 43).points_in_faces(100)
    );
}

TEST(Sampler, ValidDescriptorsAndBatches) {
    for (const auto* name : {"square.off", "torus_simple.off"}) {
        const auto mesh = Mesh::from_file(config::meshes_directory() / name);
        Sampler    sampler(mesh, 42);
        for (const auto n : {0U, 1U, 100U}) {
            for (const auto& points :
                 {sampler.points_on_vertices(n),
                  sampler.points_on_edges(n),
                  sampler.points_in_faces(n)}) {
                EXPECT_EQ(points.size(), n);
                for (const auto& p : points) {
                    EXPECT_TRUE(position(mesh, p).allFinite());
                    if (p.get_as<PointOnVertex>() == nullptr)
                        EXPECT_TRUE(face(mesh, p).is_valid());
                }
            }
        }
        EXPECT_THROW(
                (void)sampler.unit_tangent(sampler.point_on_vertex()),
                std::runtime_error
        );
        EXPECT_THROW(
                (void)sampler.unit_tangent(sampler.point_on_edge()), std::runtime_error
        );
    }
}

TEST(Sampler, EmptyMeshThrows) {
    auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    mesh.cgal()._mesh.clear();
    EXPECT_THROW((Sampler(mesh, 42)), std::runtime_error);
}

}  // namespace mdv::mesh
