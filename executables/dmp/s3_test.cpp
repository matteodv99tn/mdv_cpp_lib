#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sstream>
#include <string>

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_line.hpp>
#include <rerun/archetypes/series_point.hpp>
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
        f_lrnd.row(i) = dmp.eval_weighted_basis(dmp.time_to_s(dem[i].t()));

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

    rerun::RecordingStream rec("s3_test");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    // clang-format off
    // Position styling
    rec.log_static("demonstration/pos/qw", SeriesPoint().with_color(c1).with_name("qw").with_marker_size(1.5));
    rec.log_static("demonstration/pos/qx", SeriesPoint().with_color(c2).with_name("qx").with_marker_size(1.5));
    rec.log_static("demonstration/pos/qy", SeriesPoint().with_color(c3).with_name("qy").with_marker_size(1.5));
    rec.log_static("demonstration/pos/qz", SeriesPoint().with_color(c4).with_name("qz").with_marker_size(1.5));
    rec.log_static("execution/pos/qw", SeriesLine().with_color(c1).with_name(""));
    rec.log_static("execution/pos/qx", SeriesLine().with_color(c2).with_name(""));
    rec.log_static("execution/pos/qy", SeriesLine().with_color(c3).with_name(""));
    rec.log_static("execution/pos/qz", SeriesLine().with_color(c4).with_name(""));
    // Velocity styling
    rec.log_static("demonstration/vel/v1", SeriesPoint().with_color(c1).with_name("v[1]").with_marker_size(1.5));
    rec.log_static("demonstration/vel/v2", SeriesPoint().with_color(c2).with_name("v[2]").with_marker_size(1.5));
    rec.log_static("demonstration/vel/v3", SeriesPoint().with_color(c3).with_name("v[3]").with_marker_size(1.5));
    rec.log_static("demonstration/vel/v4", SeriesPoint().with_color(c4).with_name("v[4]").with_marker_size(1.5));
    rec.log_static("execution/vel/v1", SeriesLine().with_color(c1).with_name(""));
    rec.log_static("execution/vel/v2", SeriesLine().with_color(c2).with_name(""));
    rec.log_static("execution/vel/v3", SeriesLine().with_color(c3).with_name(""));
    rec.log_static("execution/vel/v4", SeriesLine().with_color(c4).with_name(""));
    // Acceleration styling
    rec.log_static("demonstration/acc/v1", SeriesPoint().with_color(c1).with_name("a[1]").with_marker_size(1.5));
    rec.log_static("demonstration/acc/v2", SeriesPoint().with_color(c2).with_name("a[2]").with_marker_size(1.5));
    rec.log_static("demonstration/acc/v3", SeriesPoint().with_color(c3).with_name("a[3]").with_marker_size(1.5));
    rec.log_static("demonstration/acc/v4", SeriesPoint().with_color(c4).with_name("a[4]").with_marker_size(1.5));
    rec.log_static("execution/acc/v1", SeriesLine().with_color(c1).with_name(""));
    rec.log_static("execution/acc/v2", SeriesLine().with_color(c2).with_name(""));
    rec.log_static("execution/acc/v3", SeriesLine().with_color(c3).with_name(""));
    rec.log_static("execution/acc/v4", SeriesLine().with_color(c4).with_name(""));
    // Forcing terms styling
    rec.log_static("forcing/desired/f1", SeriesPoint().with_color(c1).with_name("f[1]").with_marker_size(1.5));
    rec.log_static("forcing/desired/f2", SeriesPoint().with_color(c2).with_name("f[2]").with_marker_size(1.5));
    rec.log_static("forcing/desired/f3", SeriesPoint().with_color(c3).with_name("f[3]").with_marker_size(1.5));
    rec.log_static("forcing/desired/f4", SeriesPoint().with_color(c4).with_name("f[4]").with_marker_size(1.5));
    rec.log_static("forcing/learned/f1", SeriesLine().with_color(c1).with_name("f[1]"));
    rec.log_static("forcing/learned/f2", SeriesLine().with_color(c2).with_name("f[2]"));
    rec.log_static("forcing/learned/f3", SeriesLine().with_color(c3).with_name("f[3]"));
    rec.log_static("forcing/learned/f4", SeriesLine().with_color(c4).with_name("f[4]"));
    // Other stylings
    rec.log_static("coord_system", SeriesLine().with_color(c2).with_name("s"));

    for(auto i = 0; i < dmp.n_basis(); ++i)
        rec.log_static("basis/c" + std::to_string(i+1), SeriesLine().with_name(""));

    for (long i{0}; i < size(dem); ++i) {
        const double s = dmp.time_to_s(dem[i].t());
        // Export time
        rec.set_time_sequence("tick", i);
        rec.set_time_seconds("time", mdv::convert::seconds(dem[i].t()));
        // Export posititon
        rec.log("demonstration/pos/qw", Scalar(dem[i].y().w()));
        rec.log("demonstration/pos/qx", Scalar(dem[i].y().x()));
        rec.log("demonstration/pos/qy", Scalar(dem[i].y().y()));
        rec.log("demonstration/pos/qz", Scalar(dem[i].y().z()));
        rec.log("execution/pos/qw", Scalar(out[i].y().w()));
        rec.log("execution/pos/qx", Scalar(out[i].y().x()));
        rec.log("execution/pos/qy", Scalar(out[i].y().y()));
        rec.log("execution/pos/qz", Scalar(out[i].y().z()));
        // Export velocity
        rec.log("demonstration/vel/v1", Scalar(dem[i].yd()(0)));
        rec.log("demonstration/vel/v2", Scalar(dem[i].yd()(1)));
        rec.log("demonstration/vel/v3", Scalar(dem[i].yd()(2)));
        rec.log("demonstration/vel/v4", Scalar(dem[i].yd()(3)));
        rec.log("execution/vel/v1", Scalar(out[i].yd()(0)));
        rec.log("execution/vel/v2", Scalar(out[i].yd()(1)));
        rec.log("execution/vel/v3", Scalar(out[i].yd()(2)));
        rec.log("execution/vel/v4", Scalar(out[i].yd()(3)));
        // Export acceleration
        rec.log("demonstration/acc/v1", Scalar(dem[i].ydd()(0)));
        rec.log("demonstration/acc/v2", Scalar(dem[i].ydd()(1)));
        rec.log("demonstration/acc/v3", Scalar(dem[i].ydd()(2)));
        rec.log("demonstration/acc/v4", Scalar(dem[i].ydd()(3)));
        rec.log("execution/acc/v1", Scalar(out[i].ydd()(0)));
        rec.log("execution/acc/v2", Scalar(out[i].ydd()(1)));
        rec.log("execution/acc/v3", Scalar(out[i].ydd()(2)));
        rec.log("execution/acc/v4", Scalar(out[i].ydd()(3)));
        // Export forcing term
        rec.log("forcing/desired/f1", Scalar(f_des(i, 0)));
        rec.log("forcing/desired/f2", Scalar(f_des(i, 1)));
        rec.log("forcing/desired/f3", Scalar(f_des(i, 2)));
        rec.log("forcing/desired/f4", Scalar(f_des(i, 3)));
        rec.log("forcing/learned/f1", Scalar(f_lrnd(i, 0) * s));
        rec.log("forcing/learned/f2", Scalar(f_lrnd(i, 1) * s));
        rec.log("forcing/learned/f3", Scalar(f_lrnd(i, 2) * s));
        rec.log("forcing/learned/f4", Scalar(f_lrnd(i, 3) * s));
        // Other exports
        rec.log("coord_system", Scalar(s));

        // Export basis
        const Eigen::VectorXd b = dmp.eval_basis(s);
        for(auto i = 0; i < b.size();++i)
            rec.log("basis/c" + std::to_string(i+1), Scalar(b(i)));

    }
    // clang-format on
#endif  // MDV_WITH_RERUN_SDK

    return 0;
}
