#include "mdv/mesh/sampler.hpp"

#include <CGAL/boost/graph/Euler_operations.h>
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>
#include <numeric>

#include "mdv/config.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"

namespace mdv::mesh {

namespace {
    Mesh
    unequal_triangles() {
        auto mesh = Mesh::from_file(config::meshes_directory() / "square.off");
        // Areas 1/2 and 19/2, sharing the diagonal from (1,0) to (0,1).
        mesh.cgal()._mesh.point(internal::to_cgal(VertexId{2})) = {10, 10, 0};
        mesh.scale(1.0);  // Rebuild spatial caches after changing the test geometry.
        return mesh;
    }

    void
    expect_fit(
            const std::vector<int>&    counts,
            const std::vector<double>& weights,
            double                     limit
    ) {
        ASSERT_EQ(counts.size(), weights.size());
        const double total       = std::accumulate(counts.begin(), counts.end(), 0.0);
        const double weight_sum  = std::accumulate(weights.begin(), weights.end(), 0.0);
        double       chi_squared = 0.0;
        for (std::size_t i = 0; i < counts.size(); ++i) {
            const double expected = total * weights[i] / weight_sum;
            chi_squared += std::pow(counts[i] - expected, 2) / expected;
        }
        EXPECT_LT(chi_squared, limit);
    }
}  // namespace

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

TEST(Sampler, GeometricElementWeights) {
    const auto       mesh = unequal_triangles();
    const auto&      m    = internal::cgal_mesh(mesh);
    Sampler          sampler(mesh, 42);
    std::vector<int> faces(2);
    std::vector<int> vertices(4);
    std::vector<int> edges(m.num_edges());
    double           t_sum = 0.0;
    constexpr int    n     = 40000;
    for (int i = 0; i < n; ++i) {
        ++faces[sampler.face().value];
        ++vertices[sampler.point_on_vertex().v.value];
        const auto p = sampler.point_on_edge();
        ++edges[CGAL::edge(internal::to_cgal(p.he), m).idx()];
        EXPECT_TRUE(face(mesh, p.he).is_valid());
        EXPECT_GE(p.t, 1e-6);
        EXPECT_LE(p.t, 1.0 - 1e-6);
        t_sum += p.t;
    }
    // Conservative chi-square limits, with 1, 3 and 4 degrees of freedom.
    expect_fit(faces, {0.5, 9.5}, 25.0);
    expect_fit(vertices, {0.5, 10.0, 9.5, 10.0}, 35.0);
    std::vector<double> lengths;
    for (const auto e : m.edges()) {
        const auto h     = CGAL::halfedge(e, m);
        const auto delta = m.point(CGAL::target(h, m)) - m.point(CGAL::source(h, m));
        lengths.push_back(std::sqrt(delta.squared_length()));
    }
    expect_fit(edges, lengths, 40.0);
    EXPECT_NEAR(t_sum / n, 0.5, 0.01);
}

TEST(Sampler, UniformBarycentricsAndRoundTrip) {
    const auto       mesh = unequal_triangles();
    Sampler          sampler(mesh, 73);
    Vec3d            sum = Vec3d::Zero();
    std::vector<int> regions(8);
    constexpr int    n = 20000;
    for (int i = 0; i < n; ++i) {
        const auto  p = sampler.point_in_face();
        const Vec3d b = p.coords();
        EXPECT_GE(b.minCoeff(), 1e-6);
        EXPECT_NEAR(b.sum(), 1.0, 1e-14);
        sum += b;
        std::size_t region = 3;
        for (std::size_t j = 0; j < 3; ++j)
            if (p.bary[j] > 0.5) region = j;
        ++regions[4 * static_cast<std::size_t>(p.f.value) + region];
        const auto projected = from_cartesian(mesh, position(mesh, p));
        ASSERT_NE(projected.get_as<PointInFace>(), nullptr);
        EXPECT_EQ(projected.get_as<PointInFace>()->f, p.f);
        EXPECT_NEAR((position(mesh, projected) - position(mesh, p)).norm(), 0.0, 1e-12);
    }
    for (int j = 0; j < 3; ++j) EXPECT_NEAR(sum[j] / n, 1.0 / 3.0, 0.01);
    expect_fit(regions, {0.5, 0.5, 0.5, 0.5, 9.5, 9.5, 9.5, 9.5}, 50.0);
}

TEST(Sampler, UniformTangentAngles) {
    auto            mesh      = unequal_triangles();
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.rotate(Eigen::AngleAxisd(0.7, Vec3d{1, 2, 3}.normalized()));
    mesh.transform(transform);
    Sampler          sampler(mesh, 91);
    const Point      p             = sampler.point_in_face();
    const auto       f             = face(mesh, p);
    const Vec3d      normal_vector = normal(mesh, f);
    const auto       vs            = vertices(mesh, f);
    const Vec3d      u = (position(mesh, vs[1]) - position(mesh, vs[0])).normalized();
    const Vec3d      v = normal_vector.cross(u);
    std::vector<int> angles(24);
    for (int i = 0; i < 60000; ++i) {
        const auto  tangent = sampler.unit_tangent(p);
        const Vec3d d       = tangent.cartesian_vector();
        EXPECT_EQ(tangent.application_point(), p);
        EXPECT_NEAR(d.norm(), 1.0, 1e-12);
        EXPECT_NEAR(d.dot(normal_vector), 0.0, 1e-12);
        double theta = std::atan2(d.dot(v), d.dot(u));
        if (theta < 0.0) theta += 2.0 * std::numbers::pi;
        ++angles[static_cast<std::size_t>(theta * 24 / (2.0 * std::numbers::pi))];
    }
    expect_fit(angles, std::vector<double>(24, 1.0), 70.0);
}

TEST(Sampler, SkipsRemovedElements) {
    auto  mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    auto& m    = mesh.cgal()._mesh;
    CGAL::Euler::remove_face(CGAL::halfedge(internal::to_cgal(FaceId{0}), m), m);
    ASSERT_TRUE(m.has_garbage());
    mesh.scale(1.0);
    Sampler sampler(mesh, 42);
    for (int i = 0; i < 1000; ++i) {
        EXPECT_EQ(sampler.face(), FaceId{1});
        EXPECT_EQ(sampler.point_in_face().f, FaceId{1});
        const auto v = internal::to_cgal(sampler.point_on_vertex().v);
        const auto h = internal::to_cgal(sampler.point_on_edge().he);
        EXPECT_FALSE(m.is_removed(v));
        EXPECT_FALSE(m.is_removed(h));
        EXPECT_EQ(CGAL::face(h, m), internal::to_cgal(FaceId{1}));
    }
}

TEST(Sampler, ZeroAreaElementsAndMesh) {
    auto  mesh = Mesh::from_file(config::meshes_directory() / "square.off");
    auto& m    = mesh.cgal()._mesh;
    // Collapse only the first triangle. Its zero-length edge and its exclusive
    // vertex have zero probability, while the second triangle remains sampleable.
    m.point(internal::to_cgal(VertexId{0})) = m.point(internal::to_cgal(VertexId{1}));
    Sampler sampler(mesh, 42);
    for (int i = 0; i < 1000; ++i) {
        EXPECT_EQ(sampler.face(), FaceId{1});
        EXPECT_NE(sampler.point_on_vertex().v, VertexId{0});
        EXPECT_GT(direction(mesh, sampler.point_on_edge().he).norm(), 0.0);
    }
    for (const auto v : m.vertices()) m.point(v) = {0, 0, 0};
    EXPECT_THROW((Sampler(mesh, 42)), std::runtime_error);
}

}  // namespace mdv::mesh
