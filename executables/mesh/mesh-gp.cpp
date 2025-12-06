#include <Eigen/Core>
#include <fmt/base.h>

#include <rerun/archetypes/arrows3d.hpp>
#include <rerun/components/vector3d.hpp>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#define MATIOCPP_HAS_EIGEN

#include <fmt/os.h>
#include <iostream>
#include <matioCpp/EigenConversions.h>
#include <matioCpp/matioCpp.h>
#include <string>

#include <range/v3/all.hpp>
#include <rerun.hpp>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/kernel.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/mesh_utilities.hpp"
#include "mdv/rerun.hpp"

using namespace mdv::mesh;
namespace rs = ::ranges;
namespace rv = ::ranges::views;

using std::filesystem::path;

using PointVector = std::vector<Point>;

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
get_demonstration_data(const matioCpp::CellArray& demos, const std::size_t demo_id) {
    matioCpp::Struct d0 = demos(demo_id).asStruct();
    return {0.1 * matioCpp::to_eigen(d0["pos"].asMultiDimensionalArray<double>()),
            0.1 * matioCpp::to_eigen(d0["vel"].asMultiDimensionalArray<double>())};
}

std::pair<long, long>
find_start_end_indices(const Eigen::MatrixXd& velocities) {
    auto start_id = std::find_if(
            velocities.colwise().begin(),
            velocities.colwise().end(),
            [](const auto& v) -> bool { return v.norm() > 1e-3; }
    );

    auto end_id = std::find_if(
            velocities.colwise().rbegin(),
            velocities.colwise().rend(),
            [](const auto& v) -> bool { return v.norm() > 1e-3; }
    );

    return {std::distance(velocities.colwise().begin(), start_id),
            velocities.cols() - std::distance(velocities.colwise().rbegin(), end_id)
                    - 1};
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
get_trimmed_demonstration(const matioCpp::CellArray& demos, const std::size_t demo_id) {
    const auto [pos, vel]         = get_demonstration_data(demos, demo_id);
    const auto [start_id, end_id] = find_start_end_indices(vel);
    return {pos.middleCols(start_id, end_id - start_id + 1),
            vel.middleCols(start_id, end_id - start_id + 1)};
}

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
    points.reserve(pos_data.cols());
    cart_points.reserve(pos_data.cols());
    for (long i = 0; i < pos_data.cols(); i += increment) {
        const double x = pos_data(0, i);
        const double y = pos_data(1, i);
        cart_points.emplace_back(x, y, fun(x, y));
        points.emplace_back(Point::from_cartesian(mesh, {x, y, fun(x, y)}));

        mdv::Vec3d v = {vel_data(0, i), vel_data(1, i), 0.0};
        const auto n = points.back().face().normal();
        velocities.emplace_back((mdv::Mat3d::Identity() - n * n.transpose()) * v);
    }
    return {points, cart_points, velocities};
}

int
main(int argc, char* argv[]) {
    // std::string mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    // const auto  mesh      = Mesh::from_file(mesh_path);

    const std::string matfile = mdv::config::dataset_directory() / "C.mat";

    matioCpp::File      letter_dataset(matfile);
    matioCpp::CellArray demos = letter_dataset.read("demos").asCellArray();

    const auto func     = [](double x, double y) -> double { return x * x + y * y; };
    const auto meshfile = mdv::mesh::create_from_function(func);
    auto       mesh     = Mesh::from_file(meshfile);
    MeshKernel kernel(mesh);

    // Dataset generation
    std::vector<PointVector>             point_data;
    std::vector<std::vector<mdv::Vec3d>> vel_data;
    for (std::size_t i = 0; i < 5; ++i) {
        const auto [raw_pos, raw_vel] = get_trimmed_demonstration(demos, i);
        auto [pts, geod, vels] = to_demonstration_data(raw_pos, raw_vel, mesh, func, 6);
        for (std::size_t j = 0; j < vels.size() - 1; ++j)
            vels[j] = geod[j + 1] - geod[j];
        vels[vels.size() - 1].setZero();

        point_data.emplace_back(pts);
        vel_data.emplace_back(vels);
    }

    PointVector             pts  = rv::join(point_data) | rs::to_vector;
    std::vector<mdv::Vec3d> vels = rv::join(vel_data) | rs::to_vector;

    fmt::println("Number of train points: {}", pts.size());
    double max_ls = kernel.find_pointset_max_lengthscale(pts);
    fmt::println("Max ls: {}", max_ls);

    Eigen::VectorXd vx(vels.size());
    Eigen::VectorXd vy(vels.size());
    Eigen::VectorXd vz(vels.size());

    for (long i = 0; i < vels.size(); ++i) {
        vx(i) = vels[i](0);
        vy(i) = vels[i](1);
        vz(i) = vels[i](2);
    }

    // std::cout << vx << std::endl << std::endl;
    // std::cout << vy << std::endl << std::endl;
    // std::cout << vz << std::endl << std::endl;

    Expects(vx.rows() == pts.size());

    const double ls = 0.05;
    fmt::println("Evaluating kernel with lengthscale {}", ls);
    if (ls > max_ls) throw std::runtime_error("Exceeding condition");
    const Eigen::MatrixXd Kxx = kernel.squared_exponential(pts, pts, ls);

    fmt::println("Inverting matrix");
    const Eigen::MatrixXd Kxx_inv =
            (1.0 * Eigen::MatrixXd::Identity(Kxx.rows(), Kxx.cols()) + Kxx).inverse();


    mdv::RerunConverter    to_rerun;
    rerun::RecordingStream rec("gaussianprocess");
    rec.spawn().exit_on_failure();
    rec.log_static("mesh", to_rerun(mesh));

#if 1
    // PointVector test_points;
    // test_points.reserve(mesh.num_vertices());
    // for (std::size_t i = 0; i < mesh.num_vertices(); ++i)
    //     test_points.emplace_back(mesh.vertex(i));
    PointVector test_points = pts;

    fmt::println(
            "Computing cross-correlation matrix with {} test points", test_points.size()
    );
    const Eigen::MatrixXd Kxp = kernel.squared_exponential(test_points, pts, ls);

    Expects(Kxp.cols() == Kxx_inv.rows());

    const Eigen::MatrixXd vx_gp = Kxp * Kxx_inv * vx;
    const Eigen::MatrixXd vy_gp = Kxp * Kxx_inv * vy;
    const Eigen::MatrixXd vz_gp = Kxp * Kxx_inv * vz;

    std::vector<::rerun::components::Vector3D>   vecs;
    std::vector<::rerun::components::Position3D> origs;
    std::vector<::rerun::components::Vector3D>   train_vecs;
    std::vector<::rerun::components::Position3D> train_origs;
    for (long i = 0; i < test_points.size(); ++i) {
        // fmt::println("{} {} {}", vx_gp(i), vy_gp(i), vz_gp(i));
        mdv::Vec3d v{vx_gp(i), vy_gp(i), vz_gp(i)};
        v *= 5.0;

        if (v.norm() > 0.5) {
            v.normalize();
            v *= 0.5;
        }


        vecs.emplace_back(v(0), v(1), v(2));
        const auto pos = test_points[i].position();
        origs.emplace_back(pos(0), pos(1), pos(2));

        train_origs.emplace_back(pos(0), pos(1), pos(2));
        train_vecs.emplace_back(vx(i), vy(i), vz(i));
    }

    rec.log_static(
            "regressed_field",
            ::rerun::archetypes::Arrows3D::from_vectors(std::move(vecs))
                    .with_origins(std::move(origs))
    );
    rec.log_static(
            "training_field",
            ::rerun::archetypes::Arrows3D::from_vectors(std::move(train_vecs))
                    .with_origins(std::move(train_origs))
    );
#endif

    Geodesic train_pts;
    for (const auto& pt : pts) train_pts.emplace_back(pt.position());
    rec.log_static("training_data", to_rerun.as_points(train_pts));

    // Integration
    Point pos = Point::from_cartesian(
            mesh, pts[0].position() + 0.01 * mdv::Vec3d::Random()
    );
    const double dt = 0.7;

    Geodesic path;
    path.reserve(100);

    for (long i = 0; i < 200; ++i) {
        std::cout << "Iter " << i << " | ";
        rec.set_time_sequence("tick", i);

        const Eigen::MatrixXd Kxp = kernel.squared_exponential({pos}, pts, ls);
        // std::cout << "Kxx norm: " << Kxx.norm() << std::endl;
        // std::cout << "Kxx_inv norm: " << Kxx_inv.norm() << std::endl;
        // std::cout << "Kxp norm: " << Kxp.norm() << std::endl;
        const Eigen::MatrixXd vx_curr = Kxp * Kxx_inv * vx;
        const Eigen::MatrixXd vy_curr = Kxp * Kxx_inv * vy;
        const Eigen::MatrixXd vz_curr = Kxp * Kxx_inv * vz;
        assert(vx_curr.cols() == 1 && vx_curr.rows() == 1);
        assert(vy_curr.cols() == 1 && vy_curr.rows() == 1);
        assert(vz_curr.cols() == 1 && vz_curr.rows() == 1);
        mdv::Vec3d u{vx_curr(0, 0), vy_curr(0, 0), vz_curr(0, 0)};
        if (u.norm() > 1e5) {
            std::cout << "U Big! " << u.transpose() << " - norm: " << u.norm() << " | ";
            u.setZero();
        }


        std::cout << u.transpose();


        const auto       n = pos.face().normal();
        const mdv::Vec3d v = (mdv::Mat3d::Identity() - n * n.transpose()) * u;
        std::cout << " --> " << v.transpose() << "\n";
        TangentVector tv(pos, dt * v);
        pos = exponential_map(tv);
        path.emplace_back(pos.position());

        rec.log("position", to_rerun(pos));
        const auto cart_pos = pos.position();
        rec.log("velocity_cmd/x", rerun::Scalars({u(0)}));
        rec.log("velocity_cmd/y", rerun::Scalars({u(1)}));
        rec.log("velocity_cmd/z", rerun::Scalars({u(2)}));
        rec.log("pos/x", rerun::Scalars({cart_pos(0)}));
        rec.log("pos/y", rerun::Scalars({cart_pos(1)}));
        rec.log("pos/z", rerun::Scalars({cart_pos(2)}));
        rec.log("path", to_rerun(path));
    }


    return 0;
}
