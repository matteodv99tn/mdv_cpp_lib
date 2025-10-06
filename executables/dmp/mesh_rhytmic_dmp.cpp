#include <Eigen/Core>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <string>

#include <rerun/recording_stream.hpp>

#include "mdv/dmp/learnable_function.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/mesh_utilities.hpp"
#include "mdv/rerun.hpp"
#include "mdv/riemann_geometry/mesh.hpp"

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#endif  // MDV_WITH_RERUN_SDK

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/rhythmic_dmp.hpp"

using namespace mdv::mesh;
using std::filesystem::path;

int
main() {
    using M    = mdv::riemann::MeshManifold;
    using Demo = mdv::Demonstration<M>;
    // using Dmp  = mdv::RhytmicDmp<M>;
    using Dmp = mdv::RhytmicDmp<M, mdv::riemann::MeshEmbedder>;

    using namespace std::chrono_literals;

    auto fun = [](const double x, const double y) -> double {
        return std::exp(-((x - 0.2) * x + (y - 0.1) * (y - 1)));
    };
    const double     cx = 0.002;
    const double     cy = 0.01;
    const mdv::Vec3d center{cx, cy, fun(cx, cy)};
    // auto fun = [](const double x, const double y) -> double { return 0.0; };

    fmt::print("Creating mesh...\n");
    const path mesh_path = mdv::mesh::create_from_function(fun);
    fmt::print("Loading mesh...\n");
    const auto mesh = Mesh::from_file(mesh_path);
    fmt::print("Loading mesh... Done\n");


    // Demonstration generation
    const long            n_samples = 101;
    const Eigen::VectorXd thetas = Eigen::VectorXd::LinSpaced(n_samples, 0, 2.0 * M_PI);
    std::vector<Point>    positions;
    positions.reserve(n_samples);
    std::transform(
            thetas.begin(),
            thetas.end(),
            std::back_inserter(positions),
            [&fun, &mesh, cx, cy](const double theta) {
                const double x = cx + 0.5 * std::cos(theta);
                const double y = cy + 0.5 * std::sin(theta);
                const double z = fun(x, y);
                return Point::from_cartesian(mesh, {x, y, z});
            }
    );
    const Demo demo = Demo::builder()
                              .assign_position(positions)
                              .velocity_automatic_differentiation()
                              .acceleration_automatic_differentiation()
                              .set_sampling_period(5ms)
                              .create();

    Dmp dmp(40);

    const auto center_pt = Point::from_cartesian(mesh, center);
    dmp.embedding().setup_from_point_and_direction(center_pt, Eigen::Vector3d::UnitX());
    dmp.learn(demo, 1.0, center_pt);

    const auto new_goal =
            Point::from_cartesian(mesh, center + 0.2 * Eigen::Vector3d::UnitX());
    dmp.embedding().setup_from_point_and_direction(new_goal, Eigen::Vector3d::UnitX());
    const Demo out = dmp.integrate(
            demo.front().y(), demo.front().yd(), new_goal, 0.5, n_samples * 5 * 2, 1ms
    );


#ifdef MDV_WITH_RERUN_SDK
    rerun::RecordingStream rec("mesh_rhytmic_dmp");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter to_rr;

    rec.log_static("mesh", to_rr(mesh));

    Geodesic demo_path;
    demo_path.reserve(demo.size());
    std::transform(
            demo.begin(),
            demo.end(),
            std::back_inserter(demo_path),
            [](const auto& sample) { return sample.y().position(); }
    );
    rec.log_static("demonstration_path", to_rr(demo_path));
    for (const auto& sample : demo) {
        rec.set_time_duration_secs("time", mdv::convert::seconds(sample.t()));
        const auto pos = sample.y().position();
        rec.log("timedata/demo_x", rerun::Scalars{pos(0)});
        rec.log("timedata/demo_y", rerun::Scalars{pos(1)});
        rec.log("timedata/demo_z", rerun::Scalars{pos(2)});
    }


    Geodesic out_path;
    out_path.reserve(out.size());
    std::transform(
            out.begin(),
            out.end(),
            std::back_inserter(out_path),
            [](const auto& sample) { return sample.y().position(); }
    );
    rec.log_static("integrated_path", to_rr(out_path));
    for (const auto& sample : out) {
        rec.set_time_duration_secs("time", mdv::convert::seconds(sample.t()));
        const auto pos = sample.y().position();
        rec.log("timedata/out_x", rerun::Scalars{pos(0)});
        rec.log("timedata/out_y", rerun::Scalars{pos(1)});
        rec.log("timedata/out_z", rerun::Scalars{pos(2)});
    }

#endif  // MDV_WITH_RERUN_SDK

    return 0;
}
