#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <sstream>
#include <string>

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_line.hpp>
#include <rerun/archetypes/series_point.hpp>
#include <rerun/recording_stream.hpp>
#endif  // MDV_WITH_RERUN_SDK

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/dmp.hpp"
#include "mdv/dmp/dmp_utilities.hpp"
#include "mdv/rerun.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/utils/conversions.hpp"

using Quat = Eigen::Quaterniond;

std::string
to_string(const Quat& q) {
    std::stringstream ss;
    ss << q;
    return ss.str();
}

int
main() {
    using M    = mdv::riemann::Scalar;
    using Demo = mdv::Demonstration<M>;
    using Dmp  = mdv::Dmp<M>;

    const std::size_t   n_pts = 25;
    const double        ds    = 1.0 / n_pts;
    std::vector<double> pos_data;
    pos_data.reserve(n_pts + 1);
    for (std::size_t i = 0; i < 3; ++i) pos_data.emplace_back(0.0);
    for (std::size_t i = 0; i < n_pts + 1; ++i) pos_data.emplace_back(ds * i);
    for (std::size_t i = 0; i < 5; ++i) pos_data.emplace_back(1.0);


    const Demo dem = Demo::builder(pos_data.size())
                             .set_sampling_period(std::chrono::milliseconds(500))
                             .assign_position(pos_data)
                             .velocity_automatic_differentiation()
                             .acceleration_automatic_differentiation()
                             .create();
    const auto        integration_dt    = std::chrono::milliseconds(2);
    const std::size_t integration_steps = dem.back().t() / integration_dt;

    const double alpha = 48.0;
    Dmp          dmp(alpha, alpha / 4.0, 4.0, 12);
    dmp.learn(dem);
    dmp.tau = mdv::convert::seconds(integration_dt * integration_steps);
    std::cout << "tau = " << dmp.tau << "\n";
    const Demo out = dmp.integrate(
            dem.front().y(), dem.back().y(), integration_steps * 2, integration_dt
    );

    // const Demo out =
    //         dmp.integrate(dem.front().y(), dem.back().y(), dem.size(), dem[1].t());

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

    const Color c1(237, 135, 150);
    const Color c2(166, 218, 149);
    const Color c3(138, 173, 244);
    const Color c4(238, 212, 159);

    rerun::RecordingStream rec("interpolation_test");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    // clang-format off
    rec.log_static("demonstration/pos", SeriesPoint().with_color(c1).with_name("qw").with_marker_size(1.5));
    rec.log_static("execution/pos", SeriesLine().with_color(c1).with_name(""));
    rec.log_static("demonstration/vel", SeriesPoint().with_color(c1).with_name("v[1]").with_marker_size(1.5));
    rec.log_static("execution/vel", SeriesLine().with_color(c1).with_name(""));
    rec.log_static("demonstration/acc", SeriesPoint().with_color(c1).with_name("a[1]").with_marker_size(1.5));
    rec.log_static("execution/acc", SeriesLine().with_color(c1).with_name(""));

    for (long i{0}; i < size(dem); ++i) {
        // Export time
        rec.set_time_sequence("tick", i);
        rec.set_time_seconds("time", mdv::convert::seconds(dem[i].t()));
        rec.log("demonstration/pos", Scalar(dem[i].y()));
        rec.log("demonstration/vel", Scalar(dem[i].yd()));
        rec.log("demonstration/acc", Scalar(dem[i].ydd()));
    }
    for (long i{0}; i < size(out); ++i) {
        // Export time
        rec.set_time_sequence("tick", i);
        rec.set_time_seconds("time", mdv::convert::seconds(out[i].t()));
        rec.log("execution/pos", Scalar(out[i].y()));
        rec.log("execution/vel", Scalar(out[i].yd()));
        rec.log("execution/acc", Scalar(out[i].ydd()));
    }
    // clang-format on
#endif  // MDV_WITH_RERUN_SDK


    return 0;
}
