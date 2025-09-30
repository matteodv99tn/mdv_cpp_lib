#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/base.h>
#include <sstream>
#include <string>

#include "mdv/dmp/coordinate_system/coordinate_system.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/utils/logging.hpp"
#include "mdv/utils/logging_extras.hpp"
#include "mdv/utils/spdlog.hpp"

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_line.hpp>
#include <rerun/archetypes/series_point.hpp>
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
#include "mdv/utils/spdlog.hpp"

using Quat = Eigen::Quaterniond;

std::string
to_string(const Quat& q) {
    std::stringstream ss;
    ss << q;
    return ss.str();
}

using namespace mdv::mesh;
using std::filesystem::path;

int
main() {
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
    rec.log_static("demonstration/trajectory/x", rerun::archetypes::SeriesPoint().with_color(c1).with_marker_size(demo_marker_size).with_name("demonstration x"));
    rec.log_static("demonstration/trajectory/y", rerun::archetypes::SeriesPoint().with_color(c2).with_marker_size(demo_marker_size).with_name("demonstration y"));
    rec.log_static("demonstration/trajectory/z", rerun::archetypes::SeriesPoint().with_color(c3).with_marker_size(demo_marker_size).with_name("demonstration z"));
    rec.log_static("integration/trajectory/x", rerun::archetypes::SeriesLine().with_color(c1).with_width(integration_line_width).with_name("reproduction x"));
    rec.log_static("integration/trajectory/y", rerun::archetypes::SeriesLine().with_color(c2).with_width(integration_line_width).with_name("reproduction y"));
    rec.log_static("integration/trajectory/z", rerun::archetypes::SeriesLine().with_color(c3).with_width(integration_line_width).with_name("reproduction z"));
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
    const path mesh_path = mdv::mesh::create_from_function(fun);
    fmt::print("Loading mesh...\n");
    const auto mesh = Mesh::from_file(mesh_path);
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
    std::transform(
            begin(path_coords),
            end(path_coords),
            std::back_inserter(demonstration_path),
            [&mesh, &path_f](const double t) -> mdv::mesh::Point {
                return mdv::mesh::Point::from_cartesian(mesh, path_f(t * 3.14));
            }
    );

    // Build cartesian path and visualise it
    std::vector<Eigen::Vector3d> cartesian_demonstration;
    cartesian_demonstration.reserve(demonstration_path.size());
    std::transform(
            begin(demonstration_path),
            end(demonstration_path),
            back_inserter(cartesian_demonstration),
            [](const Point& p) { return p.position(); }
    );
    rec.log_static("demonstration/path", rr_converter(cartesian_demonstration));

    const Demo demonstration = Demo::builder(demonstration_path.size())
                                       .assign_position(demonstration_path)
                                       .velocity_automatic_differentiation()
                                       .acceleration_automatic_differentiation()
                                       .set_sampling_period(dts_demo)
                                       .create();
    for (long i = 0; i < demonstration.size(); ++i) {
        rec.set_time_seconds("time", mdv::convert::seconds(demonstration[i].t()));
        // rec.set_time_sequence("tick", i);
        const Eigen::Vector3d pos = demonstration[i].y().position();
        rec.log("demonstration/trajectory/position",
                rr_converter(demonstration[i].y()));
        rec.log("demonstration/trajectory/x", rerun::Scalar(pos(0)));
        rec.log("demonstration/trajectory/y", rerun::Scalar(pos(1)));
        rec.log("demonstration/trajectory/z", rerun::Scalar(pos(2)));
    }


    const Point y0 = demonstration_path.front();
    const Point g  = demonstration_path.back();

    long ns_int = ns_demo * (dts_demo / 1ms);

    Dmp dmp(mdv::get_default_logger(), 48.0, 12.0, 3.0, 20);
    dmp.tau = 1.0;
    fmt::print("Learning...\n");
    dmp.embedding().setup(demonstration.front().y(), demonstration.back().y());
    dmp.learn(demonstration);
    fmt::print("Learning... Done!\n");
    Point new_g = Point::from_cartesian(mesh, 0.1 * y0.position() + 0.9 * g.position());
    mdv::DemonstrationSample<M, 0, double> new_g_sample;
    new_g_sample.y() = new_g;
    fmt::print("Setting up embedding\n");
    dmp.embedding().setup(demonstration.front().y(), new_g);
    fmt::print("y0: {}\n", mdv::eigen_to_str(y0.position()));
    fmt::print("g: {}\n", mdv::eigen_to_str(g.position()));
    fmt::print("g': {}\n", mdv::eigen_to_str(new_g.position()));

    fmt::print("Integrating...\n");
    const Demo out = dmp.integrate(y0, new_g, ns_int, 1ms);
    fmt::print("Integrating... Done!\n");

    std::vector<Eigen::Vector3d> res_traj(out.size());
    for (long i = 0; i < out.size(); ++i) {
        rec.set_time_seconds("time", mdv::convert::seconds(out[i].t()));
        // rec.set_time_sequence("tick", i);
        const Eigen::Vector3d pos = out[i].y().position();
        res_traj[i]               = pos;
        rec.log("integration/trajectory/position", rr_converter(out[i].y()));
        rec.log("integration/trajectory/x", rerun::Scalar(pos(0)));
        rec.log("integration/trajectory/y", rerun::Scalar(pos(1)));
        rec.log("integration/trajectory/z", rerun::Scalar(pos(2)));
    }
    rec.log_static("integration/path", rr_converter(res_traj));

    return 0;


#ifdef MDV_WITH_RERUN_SDK
    //  ____  _       _   _   _
    // |  _ \| | ___ | |_| |_(_)_ __   __ _
    // | |_) | |/ _ \| __| __| | '_ \ / _` |
    // |  __/| | (_) | |_| |_| | | | | (_| |
    // |_|   |_|\___/ \__|\__|_|_| |_|\__, |
    //                                |___/
    using rerun::Scalar;
    using rerun::archetypes::SeriesLine;
    using rerun::archetypes::SeriesPoint;
    using rerun::components::Color;


    mdv::mesh::Geodesic geod;
    geod.reserve(out.size());
    for (const auto& p : out) geod.emplace_back(p.y().position());

    rec.log_static("trajectory", rr_converter(geod).with_colors(c2));
    for (long i{0}; i < size(out); ++i) {
        // Export time
        // rec.set_time_sequence("tick", i);
        rec.set_time_seconds("time", mdv::convert::seconds(out[i].t()));
        // Export posititon
        rec.log("position",
                rr_converter(out[i].y())
                        .with_colors(c1)
                        .with_radii(rerun::components::Radius::ui_points(5)));
    }
#endif  // MDV_WITH_RERUN_SDK

    return 0;
}
