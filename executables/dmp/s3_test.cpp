#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sstream>
#include <string>

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_lines.hpp>
#include <rerun/archetypes/series_points.hpp>
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
    using M    = mdv::riemann::S3;
    using Demo = mdv::Demonstration<M>;
    using Dmp  = mdv::Dmp<mdv::riemann::S3>;

    using namespace std::chrono_literals;

    // const long ns = 501;


    const Demo dem = mdv::build_quaternion_demonstration();
    const long ns  = dem.size();


    Dmp dmp;
    dmp.learn(dem);
    const Demo out =
            dmp.integrate(dem.front().y(), dem.back().y(), dem.size(), dem[1].t());

    const Eigen::MatrixXd f_des = dmp.evaluate_desired_forcing_term(dem);
    Eigen::MatrixXd       f_lrnd(f_des.rows(), f_des.cols());
    for (auto i = 0; i < dem.size(); ++i)
        f_lrnd.row(i) = dmp.fun()(dmp.time_to_s(dem[i].t()));

#ifdef MDV_WITH_RERUN_SDK
    //  ____  _       _   _   _
    // |  _ \| | ___ | |_| |_(_)_ __   __ _
    // | |_) | |/ _ \| __| __| | '_ \ / _` |
    // |  __/| | (_) | |_| |_| | | | | (_| |
    // |_|   |_|\___/ \__|\__|_|_| |_|\__, |
    //                                |___/
    using rerun::Scalars;
    using rerun::archetypes::SeriesLines;
    using rerun::archetypes::SeriesPoints;
    using rerun::components::Color;

    const Color c1(237, 135, 150);
    const Color c2(166, 218, 149);
    const Color c3(138, 173, 244);
    const Color c4(238, 212, 159);

    rerun::RecordingStream rec("s3_test");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    // clang-format off
    // Position styling
    rec.log_static("demonstration/pos/qw", SeriesPoints().with_colors(c1).with_names("qw").with_marker_sizes(1.5));
    rec.log_static("demonstration/pos/qx", SeriesPoints().with_colors(c2).with_names("qx").with_marker_sizes(1.5));
    rec.log_static("demonstration/pos/qy", SeriesPoints().with_colors(c3).with_names("qy").with_marker_sizes(1.5));
    rec.log_static("demonstration/pos/qz", SeriesPoints().with_colors(c4).with_names("qz").with_marker_sizes(1.5));
    rec.log_static("execution/pos/qw", SeriesLines().with_colors(c1).with_names(""));
    rec.log_static("execution/pos/qx", SeriesLines().with_colors(c2).with_names(""));
    rec.log_static("execution/pos/qy", SeriesLines().with_colors(c3).with_names(""));
    rec.log_static("execution/pos/qz", SeriesLines().with_colors(c4).with_names(""));
    // Velocity styling
    rec.log_static("demonstration/vel/v1", SeriesPoints().with_colors(c1).with_names("v[1]").with_marker_sizes(1.5));
    rec.log_static("demonstration/vel/v2", SeriesPoints().with_colors(c2).with_names("v[2]").with_marker_sizes(1.5));
    rec.log_static("demonstration/vel/v3", SeriesPoints().with_colors(c3).with_names("v[3]").with_marker_sizes(1.5));
    rec.log_static("demonstration/vel/v4", SeriesPoints().with_colors(c4).with_names("v[4]").with_marker_sizes(1.5));
    rec.log_static("execution/vel/v1", SeriesLines().with_colors(c1).with_names(""));
    rec.log_static("execution/vel/v2", SeriesLines().with_colors(c2).with_names(""));
    rec.log_static("execution/vel/v3", SeriesLines().with_colors(c3).with_names(""));
    rec.log_static("execution/vel/v4", SeriesLines().with_colors(c4).with_names(""));
    // Acceleration styling
    rec.log_static("demonstration/acc/v1", SeriesPoints().with_colors(c1).with_names("a[1]").with_marker_sizes(1.5));
    rec.log_static("demonstration/acc/v2", SeriesPoints().with_colors(c2).with_names("a[2]").with_marker_sizes(1.5));
    rec.log_static("demonstration/acc/v3", SeriesPoints().with_colors(c3).with_names("a[3]").with_marker_sizes(1.5));
    rec.log_static("demonstration/acc/v4", SeriesPoints().with_colors(c4).with_names("a[4]").with_marker_sizes(1.5));
    rec.log_static("execution/acc/v1", SeriesLines().with_colors(c1).with_names(""));
    rec.log_static("execution/acc/v2", SeriesLines().with_colors(c2).with_names(""));
    rec.log_static("execution/acc/v3", SeriesLines().with_colors(c3).with_names(""));
    rec.log_static("execution/acc/v4", SeriesLines().with_colors(c4).with_names(""));
    // Forcing terms styling
    rec.log_static("forcing/desired/f1", SeriesPoints().with_colors(c1).with_names("f[1]").with_marker_sizes(1.5));
    rec.log_static("forcing/desired/f2", SeriesPoints().with_colors(c2).with_names("f[2]").with_marker_sizes(1.5));
    rec.log_static("forcing/desired/f3", SeriesPoints().with_colors(c3).with_names("f[3]").with_marker_sizes(1.5));
    rec.log_static("forcing/desired/f4", SeriesPoints().with_colors(c4).with_names("f[4]").with_marker_sizes(1.5));
    rec.log_static("forcing/learned/f1", SeriesLines().with_colors(c1).with_names("f[1]"));
    rec.log_static("forcing/learned/f2", SeriesLines().with_colors(c2).with_names("f[2]"));
    rec.log_static("forcing/learned/f3", SeriesLines().with_colors(c3).with_names("f[3]"));
    rec.log_static("forcing/learned/f4", SeriesLines().with_colors(c4).with_names("f[4]"));
    // Other stylings
    rec.log_static("coord_system", SeriesLines().with_colors(c2).with_names("s"));

    for(auto i = 0; i < dmp.n_basis(); ++i)
        rec.log_static("basis/c" + std::to_string(i+1), SeriesLines().with_names(""));

    for (long i{0}; i < size(dem); ++i) {
        const double s = dmp.time_to_s(dem[i].t());
        // Export time
        rec.set_time_sequence("tick", i);
        rec.set_time_duration_secs("time", mdv::convert::seconds(dem[i].t()));
        // Export posititon
        rec.log("demonstration/pos/qw", Scalars(dem[i].y().w()));
        rec.log("demonstration/pos/qx", Scalars(dem[i].y().x()));
        rec.log("demonstration/pos/qy", Scalars(dem[i].y().y()));
        rec.log("demonstration/pos/qz", Scalars(dem[i].y().z()));
        rec.log("execution/pos/qw", Scalars(out[i].y().w()));
        rec.log("execution/pos/qx", Scalars(out[i].y().x()));
        rec.log("execution/pos/qy", Scalars(out[i].y().y()));
        rec.log("execution/pos/qz", Scalars(out[i].y().z()));
        // Export velocity
        rec.log("demonstration/vel/v1", Scalars(dem[i].yd()(0)));
        rec.log("demonstration/vel/v2", Scalars(dem[i].yd()(1)));
        rec.log("demonstration/vel/v3", Scalars(dem[i].yd()(2)));
        rec.log("demonstration/vel/v4", Scalars(dem[i].yd()(3)));
        rec.log("execution/vel/v1", Scalars(out[i].yd()(0)));
        rec.log("execution/vel/v2", Scalars(out[i].yd()(1)));
        rec.log("execution/vel/v3", Scalars(out[i].yd()(2)));
        rec.log("execution/vel/v4", Scalars(out[i].yd()(3)));
        // Export acceleration
        rec.log("demonstration/acc/v1", Scalars(dem[i].ydd()(0)));
        rec.log("demonstration/acc/v2", Scalars(dem[i].ydd()(1)));
        rec.log("demonstration/acc/v3", Scalars(dem[i].ydd()(2)));
        rec.log("demonstration/acc/v4", Scalars(dem[i].ydd()(3)));
        rec.log("execution/acc/v1", Scalars(out[i].ydd()(0)));
        rec.log("execution/acc/v2", Scalars(out[i].ydd()(1)));
        rec.log("execution/acc/v3", Scalars(out[i].ydd()(2)));
        rec.log("execution/acc/v4", Scalars(out[i].ydd()(3)));
        // Export forcing term
        rec.log("forcing/desired/f1", Scalars(f_des(i, 0)));
        rec.log("forcing/desired/f2", Scalars(f_des(i, 1)));
        rec.log("forcing/desired/f3", Scalars(f_des(i, 2)));
        rec.log("forcing/desired/f4", Scalars(f_des(i, 3)));
        rec.log("forcing/learned/f1", Scalars(f_lrnd(i, 0) * s));
        rec.log("forcing/learned/f2", Scalars(f_lrnd(i, 1) * s));
        rec.log("forcing/learned/f3", Scalars(f_lrnd(i, 2) * s));
        rec.log("forcing/learned/f4", Scalars(f_lrnd(i, 3) * s));
        // Other exports
        rec.log("coord_system", Scalars(s));

        // Export basis
        const Eigen::VectorXd b = dmp.fun().eval_basis(s);
        for(auto i = 0; i < b.size();++i)
            rec.log("basis/c" + std::to_string(i+1), Scalars(b(i)));

    }
    // clang-format on
#endif  // MDV_WITH_RERUN_SDK

    return 0;
}
