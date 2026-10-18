#include <iostream>
#include <string>

#include <rerun/recording_stream.hpp>

#include "mdv/dmp/learnable_function.hpp"

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#endif  // MDV_WITH_RERUN_SDK

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/rhythmic_dmp.hpp"
#include "mdv/riemann_geometry/scalar.hpp"

namespace mdv::dmp {}  // namespace mdv::dmp

int
main() {
    using M    = mdv::riemann::Scalar;
    using Demo = mdv::Demonstration<M>;
    using Dmp  = mdv::RhytmicDmp<M>;

    using namespace std::chrono_literals;

    const auto basis = mdv::dmp::PeriodicExponentialBasis::create_equispaced(20);

    // Create demonstration
    const std::size_t   n_samples = 101;
    std::vector<double> pos(101);
    for (std::size_t i = 0; i < n_samples; ++i) {
        const double t = 2.0 * M_PI * double(i) / double(n_samples);
        pos[i]         = std::exp(1.0 * (std::sin(t) - 1.0));
    }
    const Demo demo = Demo::builder()
                              .assign_position(pos)
                              .velocity_automatic_differentiation()
                              .acceleration_automatic_differentiation()
                              .set_sampling_period(5ms)
                              .create();

    // Learn demonstration
    Dmp dmp(40);
    dmp.learn(demo);

    // Execute demonstration
    const Demo res = dmp.integrate(
            demo.front().y(),
            demo.front().yd() / dmp.tau,
            dmp.compute_average(demo),
            .5,
            3 * 5 * n_samples,
            1ms
    );


#ifdef MDV_WITH_RERUN_SDK
    rerun::RecordingStream rec("scalar_rhytmic_dmp");
    rec.spawn().exit_on_failure();

    // // Basis plot
    // const Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(500, 0.0, 2.0 * M_PI);
    // for (std::size_t i = 0; i < basis.size(); ++i) {
    //     const auto& b = basis[i];
    //     for (long j = 0; j < angles.size(); ++j) {
    //         rec.set_time_sequence("tick", j);
    //         rec.set_time_duration_secs("angle", angles(j));
    //         rec.log("base/" + std::to_string(i), rerun::Scalars{b(angles(j))});
    //     }
    // }

    // Demonstration plot
    for (const auto& sample : demo) {
        rec.set_time_duration_secs("time", mdv::convert::seconds(sample.t()));
        rec.log("trajectory/demonstration", rerun::Scalars{sample.y()});
    }

    // Integration plot
    for (const auto& sample : res) {
        rec.set_time_duration_secs("time", mdv::convert::seconds(sample.t()));
        rec.log("trajectory/integration", rerun::Scalars{sample.y()});
    }
#endif  // MDV_WITH_RERUN_SDK

    return 0;
}
