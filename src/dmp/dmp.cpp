#include "mdv/dmp/dmp.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <gsl/assert>

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/dmp_utilities.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/riemann_geometry/scalar.hpp"
#include "mdv/riemann_geometry/se3.hpp"

using mdv::Demonstration;
using mdv::riemann::Scalar;

namespace {
// std::vector<double> poly_5th(const Eigen::VectorXd& ts);
std::vector<double> poly_5th_1st_derivative(const Eigen::VectorXd& ts);
std::vector<double> poly_5th_2nd_derivative(const Eigen::VectorXd& ts);

}  // namespace

Demonstration<Scalar>
mdv::build_scalar_demonstration(const long n_samples) {
    using namespace std::chrono_literals;

    const std::vector<double> ts =
            poly_5th(Eigen::VectorXd::LinSpaced(n_samples, 0.0, 1.0));

    std::vector<double> pos_traj;
    for (const auto& t : ts) pos_traj.emplace_back(2 + 3 * t);

    return Demonstration<Scalar>::builder(n_samples)
            .set_sampling_period(5ms)
            .assign_position(pos_traj)
            .velocity_automatic_differentiation()
            .acceleration_automatic_differentiation()
            .create();
}

Demonstration<mdv::riemann::Rn<3>>
mdv::build_position_demonstration(const long n_samples) {
    using namespace std::chrono_literals;

    const std::vector<double> ts =
            poly_5th(Eigen::VectorXd::LinSpaced(n_samples, 0.0, 1.0));

    using Vec3 = Eigen::Vector3d;
    std::vector<Vec3> pos_traj;
    pos_traj.reserve(ts.size());
    const Vec3 start(1.0, 2.0, 3.0);
    const Vec3 delta(2.0, 0.0, -5.0);
    for (const auto& t : ts) pos_traj.emplace_back(start + delta * t);

    return Demonstration<mdv::riemann::Rn<3>>::builder(n_samples)
            .set_sampling_period(5ms)
            .assign_position(pos_traj)
            .velocity_automatic_differentiation()
            .acceleration_automatic_differentiation()
            .create();
}

Demonstration<Scalar>
mdv::build_exact_scalar_demonstration() {
    using namespace std::chrono_literals;

    const Eigen::VectorXd ts = Eigen::VectorXd::LinSpaced(1000001, 0.0, 1.0);

    const std::vector<double> pos_traj = poly_5th(ts);
    const std::vector<double> vel_traj = poly_5th_1st_derivative(ts);
    const std::vector<double> acc_traj = poly_5th_2nd_derivative(ts);

    return Demonstration<Scalar>::builder()
            .set_sampling_period(1us)
            .assign_position(pos_traj)
            .assign_velocity(vel_traj)
            .assign_acceleration(acc_traj)
            .create();
}

Demonstration<mdv::riemann::S3>
mdv::build_quaternion_demonstration(const long n_samples) {
    using mdv::riemann::S3;
    using namespace std::chrono_literals;
    using Quat = Eigen::Quaterniond;

#if 0
    const std::vector<double> ts =
            poly_5th(Eigen::VectorXd::LinSpaced(n_samples, 0.0, 1.0));
#else
    std::vector<double> ts;
    const auto          tdata = Eigen::VectorXd::LinSpaced(n_samples - 10, 0.0, 1.0);
    for (auto i = 0; i < 5; ++i) ts.push_back(0.0);
    for (auto i = 0; i < n_samples - 10; ++i) ts.push_back(tdata(i));
    for (auto i = 0; i < 5; ++i) ts.push_back(1.0);
#endif


    // const Quat q1 = Quat(Eigen::Vector4d(1.0, 3.0, -4.0, 2.0).normalized());
    // // if (q1.w() < 0.0) q1.coeffs() *= -1.0;
    // Expects(q1.w() >= 0.0);
    // const Quat q2 = Quat::Identity();

    const Quat q1 = Quat(1.0, 0.0, 0.0, 0.0);
    const Quat q2 = Quat(0.0, -1.0, 0.0, 0.0);


    std::vector<Eigen::Quaterniond> pos_traj;
    pos_traj.reserve(n_samples);
    for (const auto& t : ts) pos_traj.emplace_back(q1.slerp(t, q2));

    return Demonstration<S3>::builder(n_samples)
            .set_sampling_period(1ms)
            .assign_position(pos_traj)
            .velocity_automatic_differentiation()
            .acceleration_automatic_differentiation()
            .create();
}

Demonstration<mdv::riemann::SE3>
mdv::build_se3_demonstration(const long n_samples) {
    using mdv::riemann::SE3;
    using namespace std::chrono_literals;

    const auto r3_dem = build_position_demonstration(n_samples);
    const auto s3_dem = build_quaternion_demonstration(n_samples);


    std::vector<SE3::Point> pos_traj;
    pos_traj.reserve(s3_dem.size());
    for (std::size_t i = 0; i < s3_dem.size(); ++i)
        pos_traj.emplace_back(r3_dem[i].y(), s3_dem[i].y());

    return Demonstration<SE3>::builder(n_samples)
            .set_sampling_period(5ms)
            .assign_position(pos_traj)
            .velocity_automatic_differentiation()
            .acceleration_automatic_differentiation()
            .create();
}

//  ____  _        _   _        _____
// / ___|| |_ __ _| |_(_) ___  |  ___|   _ _ __   ___ ___
// \___ \| __/ _` | __| |/ __| | |_ | | | | '_ \ / __/ __|
//  ___) | || (_| | |_| | (__  |  _|| |_| | | | | (__\__ \_
// |____/ \__\__,_|\__|_|\___| |_|   \__,_|_| |_|\___|___(_)
//

std::vector<double>
mdv::poly_5th(const Eigen::VectorXd& ts) {
    using std::pow;

    std::vector<double> p;
    p.reserve(ts.size());
    for (const auto& t : ts)
        p.emplace_back(10.0 * pow(t, 3) - 15.0 * pow(t, 4) + 6 * pow(t, 5));
    return p;
}

namespace {

std::vector<double>
poly_5th_1st_derivative(const Eigen::VectorXd& ts) {
    using std::pow;

    std::vector<double> p;
    p.reserve(ts.size());
    for (const auto& t : ts)
        p.emplace_back(30.0 * pow(t, 2) - 60.0 * pow(t, 3) + 30 * pow(t, 4));
    return p;
}

std::vector<double>
poly_5th_2nd_derivative(const Eigen::VectorXd& ts) {
    using std::pow;

    std::vector<double> p;
    p.reserve(ts.size());
    for (const auto& t : ts)
        p.emplace_back(60.0 * pow(t, 1) - 180.0 * pow(t, 2) + 60 * pow(t, 3));
    return p;
}

}  // namespace
