#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/base.h>
#include <string>

#include <range/v3/all.hpp>


#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_lines.hpp>
#include <rerun/archetypes/series_points.hpp>
#endif  // MDV_WITH_RERUN_SDK

#include "mdv/config.hpp"
#include "mdv/dmp/moving_dmp.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/rerun.hpp"
#include "mdv/utils/logging.hpp"
#include "mdv/utils/spdlog.hpp"

using namespace mdv::mesh;
using std::filesystem::path;

namespace rs = ::ranges;
namespace rv = ::ranges::views;

int
main() {
    // Rerun setup
    rerun::RecordingStream rec("moving-mesh");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    mdv::set_default_logger(
            mdv::static_logger_factory("MovingMeshDmp", mdv::Logger::Debug)
    );

    const std::string mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    auto              mesh      = Mesh::from_file(mesh_path);
    rec.log_static("mesh", rr_converter(mesh));
    fmt::println("Mesh loaded");

    const auto p1   = Point::from_cartesian(mesh, {-1.0, -2.1, 3.0});
    const auto p2   = Point::from_cartesian(mesh, {0.0, -0.6, 3.0});
    const auto p3   = Point::from_cartesian(mesh, {1.0, -2.1, 3.0});
    const auto g1   = mesh.build_geodesic(p1, p2);
    const auto g2   = mesh.build_geodesic(p2, p3);
    const auto geod = rv::concat(g1, g2 | rv::drop(1)) | rs::to_vector;

    rec.log_static("path", rr_converter(geod));
    fmt::println("Geodesic path computed");

    mdv::MovingDmpParameters params{
            .dt_ms = 5, .circle_radius = 0.1, .num_centroid_steps = 30
    };
    const auto traj = mdv::generate_trajectory(params, mesh, geod);

    const Geodesic path = traj
                          | rv::transform([](const Point& pt) { return pt.position(); })
                          | rs::to_vector;
    rec.log_static("planned_path", rr_converter(path));

    const auto     upsampled_traj = mdv::upsample_to_1khz(mesh, traj, params);
    const Geodesic upsampled_path =
            upsampled_traj
            | rv::transform([](const Point& pt) { return pt.position(); })
            | rs::to_vector;
    rec.log_static("upsampled_path", rr_converter(upsampled_path));

    for (long i = 0; i < traj.size(); ++i) {
        rec.set_time_sequence("step", i);
        rec.log("position", rr_converter.as_points({path[i]}));
    }


    return 0;
}
