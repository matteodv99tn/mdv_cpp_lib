#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <type_traits>

#include "mdv/config.hpp"
#include "mdv/dmp/concepts.hpp"
#include "mdv/dmp/dmp.hpp"
#include "mdv/dmp/rhythmic_dmp.hpp"
#include "mdv/mesh/mesh_manager.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/riemann_geometry/concepts.hpp"
#include "mdv/riemann_geometry/euclidean.hpp"
#include "mdv/riemann_geometry/fwd.hpp"
#include "mdv/riemann_geometry/mesh.hpp"
#include "mdv/riemann_geometry/s3.hpp"
#include "mdv/riemann_geometry/scalar.hpp"
#include "mdv/riemann_geometry/se3.hpp"

namespace mdv::riemann {

// static_assert(
//         mdv::riemann::space_dimension_v<mdv::riemann::Scalar::TangentVector> == 1
// );
// static_assert(mdv::riemann::space_dimension_v<mdv::riemann::S3::TangentVector> == 4);
// static_assert(mdv::riemann::space_dimension_v<mdv::riemann::SE3::TangentVector> ==
// 7);

static_assert(concepts::manifold<Scalar>);
static_assert(concepts::manifold<Rn<3>>);
static_assert(concepts::manifold<S3>);
static_assert(concepts::manifold<SE3>);
static_assert(concepts::manifold<MeshManifold>);
static_assert(!std::is_default_constructible_v<MeshManifold>);
static_assert(demonstration_manifold_type<MeshManifold>);

static_assert(concepts::euclidean_space<Scalar>);
static_assert(concepts::euclidean_space<Rn<3>>);
static_assert(!concepts::euclidean_space<S3>);
static_assert(!concepts::euclidean_space<SE3>);

static_assert(concepts::trivially_embeddable_manifold<Scalar>);
static_assert(concepts::trivially_embeddable_manifold<Rn<3>>);
static_assert(concepts::trivially_embeddable_manifold<S3>);
static_assert(concepts::trivially_embeddable_manifold<SE3>);

static_assert(
        space_dimension_v<TrivialTypeEmbedding<Scalar::TangentVector>::Output> == 1
);
static_assert(
        space_dimension_v<TrivialTypeEmbedding<Rn<3>::TangentVector>::Output> == 3
);
static_assert(space_dimension_v<TrivialTypeEmbedding<S3::TangentVector>::Output> == 4);
static_assert(space_dimension_v<TrivialTypeEmbedding<SE3::TangentVector>::Output> == 7);

TEST(MeshManifold, RejectsNullOwner) {
    EXPECT_THROW((MeshManifold{nullptr}), std::invalid_argument);
    mesh::MeshManager manager;
    EXPECT_THROW(manager.insert_mesh(nullptr), std::invalid_argument);
}

TEST(MeshManifold, OwnsMeshAfterManagerDestruction) {
    std::shared_ptr<MeshManifold>   manifold;
    std::weak_ptr<const mesh::Mesh> weak_mesh;
    {
        mesh::MeshManager manager;
        auto              owner =
                manager.get_mesh_from_file(config::meshes_directory() / "square.off");
        EXPECT_EQ(
                owner,
                manager.get_mesh_from_file(config::meshes_directory() / "square.off")
        );
        weak_mesh = owner;
        manifold  = std::make_shared<MeshManifold>(owner);
    }
    ASSERT_FALSE(weak_mesh.expired());
    EXPECT_EQ(manifold->mesh().num_faces(), 2);
    const mesh::Point p{
            manifold->mesh(), mesh::FaceId{0},
             Eigen::Vector3d{0.2, 0.3, 0.5}
    };
    EXPECT_TRUE(manifold->logarithmic_map(p, p).isZero(1e-12));
    manifold.reset();
    EXPECT_TRUE(weak_mesh.expired());
}

namespace {
    // Two triangles keep the ownership/rebinding regressions cheap and deterministic.
    std::shared_ptr<mesh::Mesh>
    square_owner() {
        return std::make_shared<mesh::Mesh>(
                mesh::Mesh::from_file(config::meshes_directory() / "square.off")
        );
    }

    Demonstration<MeshManifold>
    short_demo(const std::shared_ptr<MeshManifold>& manifold) {
        std::vector<mesh::Point> points;
        for (int i = 0; i < 5; ++i) {
            const double s = 0.01 * i;
            points.emplace_back(
                    manifold->mesh(),
                    mesh::FaceId{0},
                    Eigen::Vector3d{0.2 + s, 0.3 - s, 0.5}
            );
        }
        return Demonstration<MeshManifold>::builder(manifold)
                .assign_position(points)
                .set_sampling_period(std::chrono::milliseconds(10))
                .velocity_automatic_differentiation()
                .acceleration_automatic_differentiation()
                .create();
    }
}  // namespace

TEST(MeshManifold, SharedIdentityRebindAndDmpIntegration) {
    using DmpType =
            Dmp<MeshManifold,
                dmp::TransformationSystem<MeshManifold>,
                dmp::ExponentialCoordinateSystem,
                DefaultFunction,
                MeshEmbedder>;
    const auto            source = square_owner();
    auto                  target = square_owner();
    const Eigen::Affine3d rotation{Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY())};
    target->transform(rotation);
    auto    manifold = std::make_shared<MeshManifold>(source);
    auto    ts = std::make_shared<DmpType::TransformationSystem>(48.0, 12.0, manifold);
    DmpType dmp(
            4,
            ts,
            std::make_shared<DmpType::CoordinateSystem>(),
            get_default_logger(),
            manifold
    );
    const auto demo = short_demo(manifold);
    dmp.embedding().setup(demo.front().y(), demo.back().y());
    dmp.learn(demo);
    const Eigen::MatrixXd weights = dmp.weights();

    *manifold = MeshManifold(target);
    EXPECT_EQ(&dmp.manifold(), manifold.get());
    EXPECT_EQ(&dmp.transf_sys().manifold(), manifold.get());
    EXPECT_EQ(&dmp.manifold().mesh(), target.get());
    EXPECT_TRUE(dmp.weights().isApprox(weights, 0.0));

    const mesh::Point y0{
            *target, mesh::FaceId{0},
             Eigen::Vector3d{0.2, 0.3, 0.5}
    };
    const mesh::Point goal{
            *target, mesh::FaceId{0},
             Eigen::Vector3d{0.24, 0.26, 0.5}
    };
    dmp.embedding().setup(y0, goal);
    const auto result = dmp.integrate(y0, goal, 4, std::chrono::milliseconds(1));
    ASSERT_EQ(result.size(), 4);
    const auto initial = mesh::position(*target, y0);
    const auto n       = mesh::normal(*target, mesh::FaceId{0});
    EXPECT_GT((mesh::position(*target, result.back().y()) - initial).norm(), 0.0);
    for (const auto& sample : result) {
        EXPECT_TRUE(mesh::position(*target, sample.y()).allFinite());
        EXPECT_NEAR(n.dot(sample.yd()), 0.0, 1e-10);
        EXPECT_NEAR(n.dot(mesh::position(*target, sample.y()) - initial), 0.0, 1e-10);
    }
}

TEST(MeshManifold, RhythmicRebindAndIntegration) {
    using DmpType    = RhytmicDmp<MeshManifold, MeshEmbedder>;
    auto    manifold = std::make_shared<MeshManifold>(square_owner());
    auto    ts = std::make_shared<DmpType::TransformationSystem>(20.0, 5.0, manifold);
    DmpType dmp(4, ts, get_default_logger(), manifold);
    const auto demo = short_demo(manifold);
    const auto goal = demo.back().y();
    dmp.embedding().setup(demo.front().y(), goal);
    dmp.learn(demo, 0.1, goal);

    auto                  target = square_owner();
    const Eigen::Affine3d rotation{Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY())};
    target->transform(rotation);
    *manifold = MeshManifold(target);
    EXPECT_EQ(&dmp.manifold(), &dmp.transf_sys().manifold());
    const mesh::Point y0{
            *target, mesh::FaceId{0},
             Eigen::Vector3d{0.2, 0.3, 0.5}
    };
    const mesh::Point g{
            *target, mesh::FaceId{0},
             Eigen::Vector3d{0.24, 0.26, 0.5}
    };
    dmp.embedding().setup(y0, g);
    const Eigen::Vector3d zero   = Eigen::Vector3d::Zero();
    const auto            dt     = std::chrono::milliseconds(1);
    const auto            result = dmp.integrate(y0, zero, g, 0.1, 3, dt);
    const auto [next, velocity]  = dmp.integrate_once(y0, zero, g, 0.1, dt, dt * 0);
    ASSERT_EQ(result.size(), 3);
    EXPECT_TRUE(
            mesh::position(*target, next)
                    .isApprox(mesh::position(*target, result[1].y()), 1e-10)
    );
    EXPECT_TRUE(velocity.isApprox(result[1].yd(), 1e-10));
    EXPECT_GT(
            (mesh::position(*target, next) - mesh::position(*target, y0)).norm(), 0.0
    );
    const auto n = mesh::normal(*target, mesh::FaceId{0});
    for (const auto& sample : result) {
        EXPECT_TRUE(mesh::position(*target, sample.y()).allFinite());
        EXPECT_NEAR(n.dot(sample.yd()), 0.0, 1e-10);
    }
}

}  // namespace mdv::riemann
