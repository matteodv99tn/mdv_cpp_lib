#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sstream>
#include <string>

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_lines.hpp>
#include <rerun/archetypes/series_points.hpp>
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

    const Demo dem = mdv::build_scalar_demonstration(151);
    Dmp        dmp;
    dmp.learn(dem);
    const Demo out =
            dmp.integrate(dem.front().y(), dem.back().y(), dem.size(), dem[1].t());

    const Eigen::VectorXd f_des = dmp.evaluate_desired_forcing_term(dem);
    Eigen::VectorXd       f_lrnd(f_des.size());
    for (auto i = 0; i < dem.size(); ++i)
        f_lrnd(i) = dmp.fun()(dmp.time_to_s(dem[i].t()));


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

    rerun::RecordingStream rec("scalar_test");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    // clang-format off
    rec.log_static("demonstration/pos", SeriesPoints().with_colors(c1).with_names("qw").with_marker_sizes(1.5));
    rec.log_static("execution/pos", SeriesLines().with_colors(c1).with_names(""));
    rec.log_static("demonstration/vel", SeriesPoints().with_colors(c1).with_names("v[1]").with_marker_sizes(1.5));
    rec.log_static("execution/vel", SeriesLines().with_colors(c1).with_names(""));
    rec.log_static("demonstration/acc", SeriesPoints().with_colors(c1).with_names("a[1]").with_marker_sizes(1.5));
    rec.log_static("execution/acc", SeriesLines().with_colors(c1).with_names(""));
    // Forcing terms styling
    rec.log_static("forcing/desired", SeriesPoints().with_colors(c1).with_names("f[1]").with_marker_sizes(1.5));
    rec.log_static("forcing/learned", SeriesLines().with_colors(c1).with_names("f[1]"));
    // Other stylings
    rec.log_static("coord_system", SeriesLines().with_colors(c2).with_names("s"));

    for(auto i = 0; i < dmp.n_basis(); ++i)
        rec.log_static("basis/c" + std::to_string(i+1), SeriesLines().with_names(""));

    for (long i{0}; i < size(dem); ++i) {
        const double s = dmp.time_to_s(dem[i].t());
        // Export time
        rec.set_time_sequence("tick", i);
        rec.set_time_duration_secs("time", mdv::convert::seconds(dem[i].t()));
        rec.log("demonstration/pos", Scalars(dem[i].y()));
        rec.log("execution/pos", Scalars(out[i].y()));
        rec.log("demonstration/vel", Scalars(dem[i].yd()));
        rec.log("execution/vel", Scalars(out[i].yd()));
        rec.log("demonstration/acc", Scalars(dem[i].ydd()));
        rec.log("execution/acc", Scalars(out[i].ydd()));

        rec.log("forcing/desired", Scalars(f_des(i)));
        rec.log("forcing/learned", Scalars(f_lrnd(i) * s));
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
