#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <sstream>
#include <string>

#include "mdv/dmp/coordinate_system/coordinate_system.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/utils/logging.hpp"
#include "mdv/utils/logging_extras.hpp"
#include "mdv/utils/spdlog.hpp"

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_lines.hpp>
#include <rerun/archetypes/series_points.hpp>
#endif  // MDV_WITH_RERUN_SDK

#include "mdv/config.hpp"
#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/dmp.hpp"
#include "mdv/dmp/dmp_utilities.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/mesh_utilities.hpp"
#include "mdv/rerun.hpp"
#include "mdv/riemann_geometry/mesh.hpp"
#include "mdv/utils/conversions.hpp"

namespace {
using Quat = Eigen::Quaterniond;

std::string
to_string(const Quat& q) {
    std::stringstream ss;
    ss << q;
    return ss.str();
}

}  // namespace

int
main() {
    using namespace mdv::mesh;
    namespace fs = std::filesystem;
    // mdv::set_default_logger(mdv::static_logger_factory("Log"));

    using namespace std::chrono_literals;
    using M = mdv::riemann::MeshManifold;

    using Demo = mdv::Demonstration<M>;
    using Dmp  = mdv::Dmp<
            M,
            mdv::dmp::TransformationSystem<M>,
            mdv::dmp::ExponentialCoordinateSystem,
            mdv::DefaultFunction,
            mdv::riemann::MeshEmbedder>;
    using Mesh = mdv::mesh::Mesh;

    const long ns_demo  = 101;  // Demonstration samples
    const auto dts_demo = 5ms;  // Demonstration sampling period

    // Rerun setup
    rerun::RecordingStream rec("meshdmp");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;


    const rerun::components::Color c1(237, 135, 150);  // red
    const rerun::components::Color c2(166, 218, 149);  // green
    const rerun::components::Color c3(138, 173, 244);  // blue
    const rerun::components::Color c4(238, 212, 159);

    const float demo_marker_size       = 1.5;
    const float integration_line_width = 1.0;

    // clang-format off
    rec.log_static("demonstration/trajectory/x", rerun::archetypes::SeriesPoints().with_colors(c1).with_marker_sizes(demo_marker_size).with_names("demonstration x"));
    rec.log_static("demonstration/trajectory/y", rerun::archetypes::SeriesPoints().with_colors(c2).with_marker_sizes(demo_marker_size).with_names("demonstration y"));
    rec.log_static("demonstration/trajectory/z", rerun::archetypes::SeriesPoints().with_colors(c3).with_marker_sizes(demo_marker_size).with_names("demonstration z"));
    rec.log_static("integration/trajectory/x", rerun::archetypes::SeriesLines().with_colors(c1).with_widths(integration_line_width).with_names("reproduction x"));
    rec.log_static("integration/trajectory/y", rerun::archetypes::SeriesLines().with_colors(c2).with_widths(integration_line_width).with_names("reproduction y"));
    rec.log_static("integration/trajectory/z", rerun::archetypes::SeriesLines().with_colors(c3).with_widths(integration_line_width).with_names("reproduction z"));
    rec.log_static("demonstration/path", rerun::archetypes::LineStrips3D().with_colors({c2}).with_radii(rerun::components::Radius::ui_points(2.5)));
    rec.log_static("integration/path",   rerun::archetypes::LineStrips3D().with_colors({c3}).with_radii(rerun::components::Radius::ui_points(2.5)));
    rec.log_static("demonstration/path", rerun::archetypes::LineStrips3D().with_colors({c2}).with_radii(rerun::components::Radius::ui_points(2.5)));
    rec.log_static("integration/path",   rerun::archetypes::LineStrips3D().with_colors({c3}).with_radii(rerun::components::Radius::ui_points(2.5)));

    // clang-format on

    auto fun = [](const double x, const double y) -> double {
        return std::exp(-((x - 0.2) * x + (y - 0.1) * (y - 1)));
    };

    // Mesh retrieval
    fmt::print("Creating mesh...\n");
    const fs::path mesh_path = mdv::mesh::create_from_function(fun);
    fmt::print("Loading mesh...\n");
    const auto  mesh_owner = std::make_shared<Mesh>(Mesh::from_file(mesh_path));
    const auto& mesh       = *mesh_owner;
    auto        m          = std::make_shared<M>(mesh_owner);
    fmt::print("Loading mesh... Done\n");
    mesh.logger().set_log_level(mdv::Logger::LogLevel::Trace);
    rec.log_static("mesh", rr_converter(mesh));

    // Demonstration generation
    auto path_f = [&fun](const double t) -> Eigen::Vector3d {
        const double xv = 0.8 * std::cos(t);
        // const double yv = 0.6 * std::sin(t + 0.1);
        const double yv = -0.6 + 0.4 * t;
        return Eigen::Vector3d{xv, yv, fun(xv, yv)};
    };
    const Eigen::VectorXd         ts = Eigen::VectorXd::LinSpaced(ns_demo, 0.0, 1.0);
    const std::vector<double>     path_coords = mdv::poly_5th(ts);
    std::vector<mdv::mesh::Point> demonstration_path;
    demonstration_path.reserve(ns_demo);
    for (const double t : path_coords) {
        demonstration_path.push_back(mdv::mesh::from_cartesian(mesh, path_f(t * 3.14)));
    }

    // Build cartesian path and visualise it
    std::vector<Eigen::Vector3d> cartesian_demonstration;
    cartesian_demonstration.reserve(demonstration_path.size());
    for (const auto& p : demonstration_path) {
        cartesian_demonstration.push_back(position(mesh, p));
    }
    rec.log_static("demonstration/path", rr_converter(cartesian_demonstration));

    const Demo demonstration = Demo::builder(m, demonstration_path.size())
                                       .assign_position(demonstration_path)
                                       .velocity_automatic_differentiation()
                                       .acceleration_automatic_differentiation()
                                       .set_sampling_period(dts_demo)
                                       .create();
    for (const auto& sample : demonstration) {
        rec.set_time_duration_secs("time", mdv::convert::seconds(sample.t()));
        // rec.set_time_sequence("tick", i);
        const Eigen::Vector3d pos = position(mesh, sample.y());
        rec.log("demonstration/trajectory/position", rr_converter(mesh, sample.y()));
        rec.log("demonstration/trajectory/x", rerun::Scalars(pos(0)));
        rec.log("demonstration/trajectory/y", rerun::Scalars(pos(1)));
        rec.log("demonstration/trajectory/z", rerun::Scalars(pos(2)));
    }


    const Point y0 = demonstration_path.front();
    const Point g  = demonstration_path.back();

    long ns_int = ns_demo * (dts_demo / 1ms);

    auto transformation = std::make_shared<Dmp::TransformationSystem>(48.0, 12.0, m);
    Dmp  dmp(12,
             transformation,
             std::make_shared<Dmp::CoordinateSystem>(),
             mdv::get_default_logger(),
             m);
    dmp.tau = 1.0;
    fmt::print("Learning...\n");
    dmp.embedding().setup(demonstration.front().y(), demonstration.back().y());
    dmp.learn(demonstration);
    fmt::print("Learning... Done!\n");
    Point new_g =
            from_cartesian(mesh, 0.1 * position(mesh, y0) + 0.9 * position(mesh, g));
    mdv::DemonstrationSample<M, 0, double> new_g_sample;
    new_g_sample.y() = new_g;
    fmt::print("Setting up embedding\n");
    dmp.embedding().setup(demonstration.front().y(), new_g);
    fmt::print("y0: {}\n", mdv::eigen_to_str(position(mesh, y0)));
    fmt::print("g: {}\n", mdv::eigen_to_str(position(mesh, g)));
    fmt::print("g': {}\n", mdv::eigen_to_str(position(mesh, new_g)));

    fmt::print("Integrating...\n");
    const Demo out = dmp.integrate(y0, new_g, static_cast<std::size_t>(ns_int), 1ms);
    fmt::print("Integrating... Done!\n");

    std::vector<Eigen::Vector3d> res_traj(out.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
        rec.set_time_duration_secs("time", mdv::convert::seconds(out[i].t()));
        // rec.set_time_sequence("tick", i);
        const Eigen::Vector3d pos = position(mesh, out[i].y());
        res_traj[i]               = pos;
        rec.log("integration/trajectory/position", rr_converter(mesh, out[i].y()));
        rec.log("integration/trajectory/x", rerun::Scalars(pos(0)));
        rec.log("integration/trajectory/y", rerun::Scalars(pos(1)));
        rec.log("integration/trajectory/z", rerun::Scalars(pos(2)));
    }
    rec.log_static("integration/path", rr_converter(res_traj));
    return 0;
}
