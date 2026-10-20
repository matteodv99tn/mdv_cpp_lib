#include <chrono>
#include <Eigen/Core>
#include <fmt/format.h>
#include <fmt/os.h>
#include <string>

#include <range/v3/all.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/arrows3d.hpp>
#include <rerun/components/vector3d.hpp>

#include "mdv/config.hpp"
#include "mdv/dataset/handwritten_dataset.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/gaussian_process.hpp"
#include "mdv/mesh/kernel.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/mesh_utilities.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/rerun.hpp"

namespace {
using namespace mdv::mesh;
namespace rs = ::ranges;
namespace rv = ::ranges::views;

using PointVector = std::vector<Point>;

std::tuple<PointVector, Geodesic, std::vector<Eigen::Vector3d>>
to_demonstration_data(
        const Eigen::MatrixXd&                       pos_data,
        const Eigen::MatrixXd&                       vel_data,
        const Mesh&                                  mesh,
        const std::function<double(double, double)>& fun,
        const long                                   increment
) {
    std::vector<Point>      points;
    std::vector<mdv::Vec3d> cart_points;
    std::vector<mdv::Vec3d> velocities;
    points.reserve(static_cast<std::size_t>(pos_data.cols()));
    cart_points.reserve(static_cast<std::size_t>(pos_data.cols()));
    for (long i = 0; i < pos_data.cols(); i += increment) {
        const double x = pos_data(0, i);
        const double y = pos_data(1, i);
        cart_points.emplace_back(x, y, fun(x, y));
        points.emplace_back(from_cartesian(mesh, {x, y, fun(x, y)}));

        mdv::Vec3d v = {vel_data(0, i), vel_data(1, i), 0.0};
        const auto n = normal(mesh, face(mesh, points.back()));
        velocities.emplace_back((mdv::Mat3d::Identity() - n * n.transpose()) * v);
    }
    return {points, cart_points, velocities};
}

}  // namespace

int
main(int argc, char* argv[]) {
    using namespace mdv::mesh;
    const double ls    = 0.05;
    const double noise = 1.0;

    const auto letter_dataset = mdv::load_letter_dataset(0.001);
    const auto dataset        = letter_dataset.at('c');


    const auto func     = [](double x, double y) -> double { return x * x + y * y; };
    const auto meshfile = mdv::mesh::create_from_function(func);
    auto       mesh     = Mesh::from_file(meshfile);
    MeshKernel kernel(mesh);

    fmt::print("Number of faces: {}\n", mesh.num_faces());
    fmt::print("Number of vertices: {}\n", mesh.num_vertices());

    // Dataset generation
    std::vector<PointVector>             point_data;
    std::vector<std::vector<mdv::Vec3d>> vel_data;
    for (std::size_t i = 0; i < 5; ++i) {
        const auto& raw_pos    = dataset.demonstrations[i].pos;
        const auto& raw_vel    = dataset.demonstrations[i].vel;
        auto [pts, geod, vels] = to_demonstration_data(raw_pos, raw_vel, mesh, func, 6);
        for (std::size_t j = 0; j < vels.size() - 1; ++j)
            vels[j] = geod[j + 1] - geod[j];
        vels[vels.size() - 1].setZero();

        point_data.emplace_back(pts);
        vel_data.emplace_back(vels);
    }

    PointVector             pts  = rv::join(point_data) | rs::to_vector;
    std::vector<mdv::Vec3d> vels = rv::join(vel_data) | rs::to_vector;

    fmt::print("Number of train points: {}\n", pts.size());
    double max_ls = kernel.find_pointset_max_lengthscale(pts);
    fmt::print("Max ls: {}\n", max_ls);

    Eigen::VectorXd vx(vels.size());
    Eigen::VectorXd vy(vels.size());
    Eigen::VectorXd vz(vels.size());


    for (std::size_t i = 0; i < vels.size(); ++i) {
        const auto k = static_cast<Eigen::Index>(i);
        vx(k)        = vels[i](0);
        vy(k)        = vels[i](1);
        vz(k)        = vels[i](2);
    }

    Eigen::MatrixXd v_mat(vels.size(), 3);
    v_mat.col(0) = vx;
    v_mat.col(1) = vy;
    v_mat.col(2) = vz;

    Expects(vx.rows() == std::ssize(pts));


    fmt::print("Creating custom Gaussian Process - Lengthscale = {}\n", ls);
    InexactGaussianProcess gp(&mesh, ls, noise);
    gp.train(pts, v_mat);
    fmt::print("Gaussian Process initialised\n");

    mdv::RerunConverter    to_rerun;
    rerun::RecordingStream rec("gaussianprocess");
    rec.spawn().exit_on_failure();
    rec.log_static("mesh", to_rerun(mesh));

    // PointVector test_points = pts;
    PointVector test_points;
    test_points.reserve(mesh.num_vertices());
    for (mdv::mesh::Index i = 0; i < mesh.num_vertices(); ++i)
        test_points.emplace_back(PointOnVertex{VertexId{i}});

    fmt::print("Predicting vector field...\n");
    const Eigen::MatrixXd v_gp = gp.predict(test_points);
    fmt::print("Prediction computed\n");

    std::vector<::rerun::components::Vector3D>   gp_vecs;
    std::vector<::rerun::components::Position3D> gp_origs;

    for (std::size_t i = 0; i < test_points.size(); ++i) {
        const auto k   = static_cast<Eigen::Index>(i);
        const auto pos = position(mesh, test_points[i]);
        gp_origs.emplace_back(pos(0), pos(1), pos(2));
        gp_vecs.emplace_back(v_gp(k, 0), v_gp(k, 1), v_gp(k, 2));
    }

    rec.log_static(
            "vector_field",
            ::rerun::archetypes::Arrows3D::from_vectors(std::move(gp_vecs))
                    .with_origins(std::move(gp_origs))
    );

    Geodesic train_pts;
    for (const auto& pt : pts) train_pts.emplace_back(position(mesh, pt));
    rec.log_static("training_data", to_rerun.as_points(train_pts));

    // Integration
    Point pos =
            from_cartesian(mesh, position(mesh, pts[0]) + 0.01 * mdv::Vec3d::Random());
    const double dt = 0.2;

    fmt::print("Starting dynamics integration\n");
    Geodesic path;
    path.reserve(100);
    auto start = std::chrono::high_resolution_clock::now();
    for (long i = 0; i < 500; ++i) {
        rec.set_time_sequence("tick", i);
        const auto v_vec = gp.predict({pos});
        mdv::Vec3d u     = v_vec.transpose();

        const auto       n = normal(mesh, face(mesh, pos));
        const mdv::Vec3d v = (mdv::Mat3d::Identity() - n * n.transpose()) * u;
        TangentVector    tv(mesh, pos, dt * v);
        pos = exponential_map(mesh, tv);
        path.emplace_back(position(mesh, pos));

        rec.log("position", to_rerun(mesh, pos));
        const auto cart_pos = position(mesh, pos);
        rec.log("velocity_cmd/x", rerun::Scalars({u(0)}));
        rec.log("velocity_cmd/y", rerun::Scalars({u(1)}));
        rec.log("velocity_cmd/z", rerun::Scalars({u(2)}));
        rec.log("pos/x", rerun::Scalars({cart_pos(0)}));
        rec.log("pos/y", rerun::Scalars({cart_pos(1)}));
        rec.log("pos/z", rerun::Scalars({cart_pos(2)}));
        rec.log("path", to_rerun(path));
    }
    auto       stop = std::chrono::high_resolution_clock::now();
    const auto time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
    fmt::print("{} steps in {}ms\n", path.size(), time_ms);


    return 0;
}
