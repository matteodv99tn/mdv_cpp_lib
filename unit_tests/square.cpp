#include <array>
#include <gtest/gtest.h>
#include <variant>

#include "mdv/config.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"

namespace mdv::mesh {
namespace fs = std::filesystem;

TEST(Mesh, Square) {
    using condition::are_equal;
    const fs::path mesh_path = mdv::config::meshes_directory() / "square.off";
    const auto     mesh      = Mesh::from_file(mesh_path);

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

TEST(Mesh, SquareProjectionAndBoundaryInvariants) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    for (Index i = 0; i < mesh.num_faces(); ++i) {
        SCOPED_TRACE(i);
        const auto face = mesh.face(i);
        for (const auto id : face.vertices_ids()) {
            SCOPED_TRACE(id);
            const HalfEdge he{mesh.vertex(id), face};
            const Vec3d    vertex    = mesh.vertex(id).position();
            const auto     on_vertex = Point::from_cartesian(mesh, vertex);
            EXPECT_EQ(location_type(on_vertex), LocationType::ON_VERTEX);
            EXPECT_NEAR((on_vertex.position() - vertex).norm(), 0.0, 1e-12);

            // Convex combinations with one/two zero weights lie on an edge/vertex.
            const Vec3d edge_position = vertex + 0.3 * he.direction();
            const auto  on_edge       = Point::from_cartesian(mesh, edge_position);
            ASSERT_EQ(location_type(on_edge), LocationType::ON_EDGE);
            EXPECT_NEAR((on_edge.position() - edge_position).norm(), 0.0, 1e-12);
            const auto& edge    = *on_edge.get_as<Point::PointOnEdgeDescriptor>();
            const auto  flipped = edge.display_in_opposite_halfedge();
            const auto  twice   = flipped.display_in_opposite_halfedge();
            EXPECT_NEAR((flipped.position() - edge.position()).norm(), 0.0, 1e-12);
            EXPECT_EQ(twice.halfedge().id(), edge.halfedge().id());
            EXPECT_NEAR((twice.position() - edge.position()).norm(), 0.0, 1e-12);
        }
    }
    for (const Vec3d query : {
                 Vec3d{0.2, 0.3, 0.0},
                 Vec3d{0.8, 0.7, 0.4}
    }) {
        const auto projected = Point::from_cartesian(mesh, query);
        EXPECT_EQ(location_type(projected), LocationType::INSIDE_FACE);
        EXPECT_NEAR(
                (projected.position() - Vec3d{query.x(), query.y(), 0.0}).norm(),
                0.0,
                1e-12
        );
        const auto again = Point::from_cartesian(mesh, projected.position());
        EXPECT_NEAR((again.position() - projected.position()).norm(), 0.0, 1e-12);
    }
}

TEST(Mesh, SquareMapAndTransportInvariants) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    const std::array<Vec3d, 3> positions{
            Vec3d{0.15, 0.2,  0.0},
            Vec3d{0.3,  0.25, 0.0},
            Vec3d{0.8,  0.65, 0.0}
    };
    for (const auto& position : positions) {
        const auto p = Point::from_cartesian(mesh, position);
        EXPECT_NEAR(logarithmic_map(p, p).cartesian_vector().norm(), 0.0, 1e-12);
        const auto zero = TangentVector::from_ambient_vector(p, Vec3d::Zero());
        EXPECT_NEAR((exponential_map(zero).position() - position).norm(), 0.0, 1e-12);
        for (const auto& target : positions) {
            const auto  q     = Point::from_cartesian(mesh, target);
            const Vec3d delta = target - position;
            const auto  log   = logarithmic_map(p, q);
            EXPECT_NEAR((log.cartesian_vector() - delta).norm(), 0.0, 1e-9);
            EXPECT_NEAR(
                    log.cartesian_vector().norm(),
                    length(mesh.build_geodesic(p, q)),
                    1e-9
            );
            Geodesic   path;
            const auto end = exponential_map(log, &path);
            EXPECT_NEAR((end.position() - target).norm(), 0.0, 1e-9);
            EXPECT_NEAR(length(path), delta.norm(), 1e-9);
            EXPECT_NEAR(length(mesh.build_geodesic(p, q)), delta.norm(), 1e-9);
        }
    }
    // Face-interior endpoints give both vectors a unique tangent plane.
    const auto          p = Point::from_cartesian(mesh, positions[0]);
    const TangentVector v(p, {0.2, 0.1, 0.0});
    const TangentVector w(p, {-0.1, 0.3, 0.0});
    for (std::size_t i = 0; i < 3; ++i) {
        const auto q  = Point::from_cartesian(mesh, positions[i]);
        const auto pv = parallel_transport(v, q);
        const auto pw = parallel_transport(w, q);
        EXPECT_NEAR((pv.cartesian_vector() - v.cartesian_vector()).norm(), 0.0, 1e-9);
        EXPECT_NEAR(pv.cartesian_vector().norm(), v.cartesian_vector().norm(), 1e-9);
        EXPECT_NEAR(
                pv.cartesian_vector().dot(pw.cartesian_vector()),
                v.cartesian_vector().dot(w.cartesian_vector()),
                1e-9
        );
        EXPECT_NEAR(
                pv.cartesian_vector().dot(pv.application_point().face().normal()),
                0.0,
                1e-9
        );
        EXPECT_NEAR(
                (pv.application_point().position() - q.position()).norm(), 0.0, 1e-9
        );
    }
}

}  // namespace mdv::mesh
