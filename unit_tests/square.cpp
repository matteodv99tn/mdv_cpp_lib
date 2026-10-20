#include <array>
#include <gtest/gtest.h>
#include <string>
#include <utility>
#include <variant>

#include "mdv/config.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh {
namespace fs = std::filesystem;

TEST(Mesh, Square) {
    using condition::are_equal;
    const fs::path mesh_path = mdv::config::meshes_directory() / "square.off";
    const auto     mesh      = Mesh::from_file(mesh_path);

    EXPECT_EQ(mesh.num_faces(), 2);
    EXPECT_EQ(mesh.num_vertices(), 4);

    EXPECT_TRUE(are_equal(Vec3d{0.0, 0.0, 0.0}, position(mesh, VertexId{0})));
    EXPECT_TRUE(are_equal(Vec3d{1.0, 0.0, 0.0}, position(mesh, VertexId{1})));
    EXPECT_TRUE(are_equal(Vec3d{1.0, 1.0, 0.0}, position(mesh, VertexId{2})));
    EXPECT_TRUE(are_equal(Vec3d{0.0, 1.0, 0.0}, position(mesh, VertexId{3})));

    const auto pt_on_v0 = from_cartesian(mesh, Vec3d{0.0, 0.0, 0.0});
    EXPECT_NO_THROW(std::get<PointOnVertex>(pt_on_v0.descriptor()));
    EXPECT_THROW(std::get<PointOnEdge>(pt_on_v0.descriptor()), std::bad_variant_access);
    EXPECT_THROW(std::get<PointInFace>(pt_on_v0.descriptor()), std::bad_variant_access);
    EXPECT_TRUE(are_equal(position(mesh, pt_on_v0), Vec3d{0.0, 0.0, 0.0}));

    const auto pt_on_edge = from_cartesian(mesh, Vec3d{0.5, 0.0, 0.0});
    EXPECT_THROW(
            std::get<PointOnVertex>(pt_on_edge.descriptor()), std::bad_variant_access
    );
    EXPECT_NO_THROW(std::get<PointOnEdge>(pt_on_edge.descriptor()));
    EXPECT_THROW(
            std::get<PointInFace>(pt_on_edge.descriptor()), std::bad_variant_access
    );
    EXPECT_TRUE(are_equal(position(mesh, pt_on_edge), Vec3d{0.5, 0.0, 0.0}));

    auto opposite_description =
            flip(mesh, std::get<PointOnEdge>(pt_on_edge.descriptor()));
    EXPECT_TRUE(are_equal(position(mesh, opposite_description), Vec3d{0.5, 0.0, 0.0}));

    const auto pt_on_diag_edge = from_cartesian(mesh, Vec3d{0.8, 0.2, 0.0});
    EXPECT_THROW(
            std::get<PointOnVertex>(pt_on_diag_edge.descriptor()),
            std::bad_variant_access
    );
    EXPECT_NO_THROW(std::get<PointOnEdge>(pt_on_diag_edge.descriptor()));
    EXPECT_THROW(
            std::get<PointInFace>(pt_on_diag_edge.descriptor()), std::bad_variant_access
    );
    EXPECT_TRUE(are_equal(position(mesh, pt_on_diag_edge), Vec3d{0.8, 0.2, 0.0}));
    opposite_description =
            flip(mesh, std::get<PointOnEdge>(pt_on_diag_edge.descriptor()));
    EXPECT_TRUE(are_equal(position(mesh, opposite_description), Vec3d{0.8, 0.2, 0.0}));

    const auto pt_in_face = from_cartesian(mesh, Vec3d{0.2, 0.2, 0.0});
    EXPECT_THROW(
            std::get<PointOnVertex>(pt_in_face.descriptor()), std::bad_variant_access
    );
    EXPECT_THROW(
            std::get<PointOnEdge>(pt_in_face.descriptor()), std::bad_variant_access
    );
    EXPECT_NO_THROW(std::get<PointInFace>(pt_in_face.descriptor()));
    EXPECT_TRUE(are_equal(position(mesh, pt_in_face), Vec3d{0.2, 0.2, 0.0}));
}

TEST(Mesh, SquareProjectionAndBoundaryInvariants) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    for (Index i = 0; i < mesh.num_faces(); ++i) {
        SCOPED_TRACE(i);
        const auto f = FaceId{i};
        for (const auto id : vertices(mesh, f)) {
            SCOPED_TRACE(id.value);
            const HalfEdgeId he        = halfedge(mesh, id, f);
            const Vec3d      vertex    = position(mesh, id);
            const auto       on_vertex = from_cartesian(mesh, vertex);
            EXPECT_EQ(location_type(on_vertex), LocationType::ON_VERTEX);
            EXPECT_NEAR((position(mesh, on_vertex) - vertex).norm(), 0.0, 1e-12);

            // Convex combinations with one/two zero weights lie on an edge/vertex.
            const Vec3d edge_position = vertex + 0.3 * direction(mesh, he);
            const auto  on_edge       = from_cartesian(mesh, edge_position);
            ASSERT_EQ(location_type(on_edge), LocationType::ON_EDGE);
            EXPECT_NEAR((position(mesh, on_edge) - edge_position).norm(), 0.0, 1e-12);
            const auto& edge    = *on_edge.get_as<PointOnEdge>();
            const auto  flipped = flip(mesh, edge);
            const auto  twice   = flip(mesh, flipped);
            EXPECT_NEAR(
                    (position(mesh, flipped) - position(mesh, edge)).norm(), 0.0, 1e-12
            );
            EXPECT_EQ(twice.he, edge.he);
            EXPECT_NEAR(
                    (position(mesh, twice) - position(mesh, edge)).norm(), 0.0, 1e-12
            );
        }
    }
    for (const Vec3d query : {
                 Vec3d{0.2, 0.3, 0.0},
                 Vec3d{0.8, 0.7, 0.4}
    }) {
        const auto projected = from_cartesian(mesh, query);
        EXPECT_EQ(location_type(projected), LocationType::INSIDE_FACE);
        EXPECT_NEAR(
                (position(mesh, projected) - Vec3d{query.x(), query.y(), 0.0}).norm(),
                0.0,
                1e-12
        );
        const auto again = from_cartesian(mesh, position(mesh, projected));
        EXPECT_NEAR(
                (position(mesh, again) - position(mesh, projected)).norm(), 0.0, 1e-12
        );
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
        const auto p = from_cartesian(mesh, position);
        EXPECT_NEAR(logarithmic_map(mesh, p, p).cartesian_vector().norm(), 0.0, 1e-12);
        const auto zero = TangentVector::from_ambient_vector(mesh, p, Vec3d::Zero());
        EXPECT_NEAR(
                (mesh::position(mesh, exponential_map(mesh, zero)) - position).norm(),
                0.0,
                1e-12
        );
        for (const auto& target : positions) {
            const auto  q     = from_cartesian(mesh, target);
            const Vec3d delta = target - position;
            const auto  log   = logarithmic_map(mesh, p, q);
            EXPECT_NEAR((log.cartesian_vector() - delta).norm(), 0.0, 1e-9);
            EXPECT_NEAR(
                    log.cartesian_vector().norm(),
                    length(build_geodesic(mesh, p, q)),
                    1e-9
            );
            Geodesic   path;
            const auto end = exponential_map(mesh, log, &path);
            EXPECT_NEAR((mesh::position(mesh, end) - target).norm(), 0.0, 1e-9);
            EXPECT_NEAR(length(path), delta.norm(), 1e-9);
            EXPECT_NEAR(length(build_geodesic(mesh, p, q)), delta.norm(), 1e-9);
        }
    }
    // Face-interior endpoints give both vectors a unique tangent plane.
    const auto          p = from_cartesian(mesh, positions[0]);
    const TangentVector v(mesh, p, {0.2, 0.1, 0.0});
    const TangentVector w(mesh, p, {-0.1, 0.3, 0.0});
    for (std::size_t i = 0; i < 3; ++i) {
        const auto q  = from_cartesian(mesh, positions[i]);
        const auto pv = parallel_transport(mesh, v, q);
        const auto pw = parallel_transport(mesh, w, q);
        EXPECT_NEAR((pv.cartesian_vector() - v.cartesian_vector()).norm(), 0.0, 1e-9);
        EXPECT_NEAR(pv.cartesian_vector().norm(), v.cartesian_vector().norm(), 1e-9);
        EXPECT_NEAR(
                pv.cartesian_vector().dot(pw.cartesian_vector()),
                v.cartesian_vector().dot(w.cartesian_vector()),
                1e-9
        );
        EXPECT_NEAR(pv.cartesian_vector().dot(normal(mesh, pv.face())), 0.0, 1e-9);
        EXPECT_NEAR(
                (position(mesh, pv.application_point()) - position(mesh, q)).norm(),
                0.0,
                1e-9
        );
    }
}

TEST(Mesh, NearbyTransportAcrossFaceBoundary) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    const auto p    = from_cartesian(mesh, Vec3d{0.4, 0.6 - 3e-7, 0.0});
    const auto q    = from_cartesian(mesh, Vec3d{0.4, 0.6 + 3e-7, 0.0});
    ASSERT_NE(face(mesh, p), face(mesh, q));
    ASSERT_TRUE(same_location(mesh, p, q));
    for (const auto& endpoints : {
                 std::pair{p, q},
                 std::pair{q, p}
    }) {
        const TangentVector v(mesh, endpoints.first, Vec3d{0.1, 0.0, 0.0});
        const auto          transported = parallel_transport(mesh, v, endpoints.second);
        EXPECT_EQ(transported.application_point(), endpoints.second);
        EXPECT_EQ(transported.face(), face(mesh, endpoints.second));
        EXPECT_NEAR(
                (transported.cartesian_vector() - v.cartesian_vector()).norm(),
                0.0,
                1e-12
        );
    }
}

TEST(Mesh, MoveAssignmentReplacesGeometry) {
    auto       source      = Mesh::from_file(config::meshes_directory() / "square.off");
    auto       destination = Mesh::from_file(config::meshes_directory() / "cube.stl");
    const auto expected_vertices = source.num_vertices();
    const auto expected_faces    = source.num_faces();
    const std::string expected_name{source.name()};
    destination = std::move(source);
    EXPECT_EQ(destination.num_vertices(), expected_vertices);
    EXPECT_EQ(destination.num_faces(), expected_faces);
    EXPECT_EQ(destination.name(), expected_name);
    auto* alias = &destination;
    destination = std::move(*alias);
    EXPECT_EQ(destination.num_vertices(), expected_vertices);
    EXPECT_EQ(destination.num_faces(), expected_faces);
    EXPECT_EQ(destination.name(), expected_name);
    auto moved = std::move(destination);
    EXPECT_EQ(moved.num_vertices(), expected_vertices);
    EXPECT_EQ(moved.num_faces(), expected_faces);
    EXPECT_EQ(moved.name(), expected_name);
}

}  // namespace mdv::mesh
