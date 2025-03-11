#include <Eigen/Dense>
#include <Eigen/Geometry>
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

    const Demo dem = mdv::build_scalar_demonstration(151);
    Dmp        dmp;
    dmp.learn(dem);
    const Demo out =
            dmp.integrate(dem.front().y(), dem.back().y(), dem.size(), dem[1].t());

    static_assert(mdv::type_elems_size_v<double> == 1);
    const Eigen::VectorXd f_des = dmp.evaluate_desired_forcing_term(dem);
    Eigen::VectorXd       f_lrnd(f_des.size());
    for (auto i = 0; i < dem.size(); ++i)
        f_lrnd(i) = dmp.eval_weighted_basis(dmp.time_to_s(dem[i].t()));


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

    rerun::RecordingStream rec("scalar_test");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    // clang-format off
    rec.log_static("demonstration/pos", SeriesPoint().with_color(c1).with_name("qw").with_marker_size(1.5));
    rec.log_static("execution/pos", SeriesLine().with_color(c1).with_name(""));
    rec.log_static("demonstration/vel", SeriesPoint().with_color(c1).with_name("v[1]").with_marker_size(1.5));
    rec.log_static("execution/vel", SeriesLine().with_color(c1).with_name(""));
    rec.log_static("demonstration/acc", SeriesPoint().with_color(c1).with_name("a[1]").with_marker_size(1.5));
    rec.log_static("execution/acc", SeriesLine().with_color(c1).with_name(""));
    // Forcing terms styling
    rec.log_static("forcing/desired", SeriesPoint().with_color(c1).with_name("f[1]").with_marker_size(1.5));
    rec.log_static("forcing/learned", SeriesLine().with_color(c1).with_name("f[1]"));
    // Other stylings
    rec.log_static("coord_system", SeriesLine().with_color(c2).with_name("s"));

    for(auto i = 0; i < dmp.n_basis(); ++i)
        rec.log_static("basis/c" + std::to_string(i+1), SeriesLine().with_name(""));

    for (long i{0}; i < size(dem); ++i) {
        const double s = dmp.time_to_s(dem[i].t());
        // Export time
        rec.set_time_sequence("tick", i);
        rec.set_time_seconds("time", mdv::convert::seconds(dem[i].t()));
        rec.log("demonstration/pos", Scalar(dem[i].y()));
        rec.log("execution/pos", Scalar(out[i].y()));
        rec.log("demonstration/vel", Scalar(dem[i].yd()));
        rec.log("execution/vel", Scalar(out[i].yd()));
        rec.log("demonstration/acc", Scalar(dem[i].ydd()));
        rec.log("execution/acc", Scalar(out[i].ydd()));

        rec.log("forcing/desired", Scalar(f_des(i)));
        rec.log("forcing/learned", Scalar(f_lrnd(i) * s));
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
