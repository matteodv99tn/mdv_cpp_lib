#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <string>

#include <range/v3/all.hpp>


#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_lines.hpp>
#include <rerun/archetypes/series_points.hpp>
#endif  // MDV_WITH_RERUN_SDK

#include "mdv/config.hpp"
#include "mdv/dmp/moving_dmp.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/rerun.hpp"
#include "mdv/utils/logging.hpp"
#include "mdv/utils/spdlog.hpp"

int
main() {
    using namespace mdv::mesh;
    namespace rs = ::ranges;
    namespace rv = ::ranges::views;
    // Rerun setup
    rerun::RecordingStream rec("moving-mesh");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    mdv::set_default_logger(
            mdv::static_logger_factory("MovingMeshDmp", mdv::Logger::Debug)
    );

    const std::string mesh_path  = mdv::config::meshes_directory() / "torus_simple.off";
    const auto        mesh_owner = std::make_shared<Mesh>(Mesh::from_file(mesh_path));
    const auto&       mesh       = *mesh_owner;
    rec.log_static("mesh", rr_converter(mesh));
    fmt::print("Mesh loaded\n");

    const auto p1 = from_cartesian(mesh, {-1.0, -2.1, 3.0});
    const auto p2 = from_cartesian(mesh, {0.0, -0.6, 3.0});
    const auto p3 = from_cartesian(mesh, {1.0, -2.1, 3.0});
    const auto g1 = build_geodesic(mesh, p1, p2);
    // const auto g2   = build_geodesic(mesh, p2, p3);
    // const auto geod = rv::concat(g1, g2 | rv::drop(1)) | rs::to_vector;
    const auto& geod = g1;

    rec.log_static("path", rr_converter(geod));
    fmt::print("Geodesic path computed\n");

    mdv::MovingDmpParameters params{
            .dt_ms = 5, .circle_radius = 0.1, .num_centroid_steps = 30
    };
    const auto traj = mdv::generate_trajectory(params, mesh_owner, geod);

    const Geodesic path =
            traj
            | rv::transform([&mesh](const Point& pt) { return position(mesh, pt); })
            | rs::to_vector;
    rec.log_static("planned_path", rr_converter(path));

    const auto     upsampled_traj = mdv::upsample_to_1khz(mesh, traj, params);
    const Geodesic upsampled_path =
            upsampled_traj
            | rv::transform([&mesh](const Point& pt) { return position(mesh, pt); })
            | rs::to_vector;
    rec.log_static("upsampled_path", rr_converter(upsampled_path));

    const auto ori =
            mdv::filter_orientation(mdv::encode_orientation(mesh, upsampled_traj), 100);

    for (std::size_t i = 0; i < upsampled_traj.size(); ++i) {
        rec.set_time_sequence("step", static_cast<std::int64_t>(i));
        rec.log("tcp", rr_converter(upsampled_path[i], ori[i]));
    }


    return 0;
}
