#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/base.h>
#include <sstream>
#include <string>

#include "mdv/dmp/coordinate_system/coordinate_system.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/riemann_geometry/se3.hpp"
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
    using namespace std::chrono_literals;
    using M = mdv::riemann::MeshManifold;

    using Se3Dmp  = mdv::Dmp<mdv::riemann::SE3>;
    using Demo    = mdv::Demonstration<mdv::riemann::SE3>;
    using MeshDmp = mdv::Dmp<
            M,
            mdv::dmp::TransformationSystem<M>,
            mdv::dmp::ExponentialCoordinateSystem,
            mdv::DefaultFunction,
            mdv::riemann::MeshEmbedder>;
    using Mesh = mdv::mesh::Mesh;

    const long ns_demo  = 101;  // Demonstration samples
    const auto dts_demo = 5ms;  // Demonstration sampling period

    // Rerun setup
    rerun::RecordingStream rec("sharp meshdmp");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;


    const rerun::components::Color c1(237, 135, 150);  // red
    const rerun::components::Color c2(166, 218, 149);  // green
    const rerun::components::Color c3(138, 173, 244);  // blue
    const rerun::components::Color c4(238, 212, 159);

    const float demo_marker_size       = 1.5;
    const float integration_line_width = 1.0;

    // clang-format off
    rec.log_static("meshdmp/trajectory/x", rerun::archetypes::SeriesLines().with_colors(c1).with_widths(integration_line_width).with_names("reproduction x"));
    rec.log_static("meshdmp/trajectory/y", rerun::archetypes::SeriesLines().with_colors(c2).with_widths(integration_line_width).with_names("reproduction y"));
    rec.log_static("meshdmp/trajectory/z", rerun::archetypes::SeriesLines().with_colors(c3).with_widths(integration_line_width).with_names("reproduction z"));
    rec.log_static("meshdmp/path",   rerun::archetypes::LineStrips3D().with_colors({c3}).with_radii(rerun::components::Radius::ui_points(5.5)));

    // clang-format on

    std::string mesh_path = mdv::config::meshes_directory() / "cube.stl";
    const auto  mesh      = Mesh::from_file(mesh_path);
    mesh.logger().set_log_level(mdv::Logger::LogLevel::Trace);
    rec.log_static("mesh", rr_converter(mesh));
    fmt::print("Mesh loaded\n");

    const Point y0 = Point::from_cartesian(mesh, {0, 50, 20});
    const Point g  = Point::from_cartesian(mesh, {100, 30, 35});

    long ns_int = ns_demo * (dts_demo / 1ms);

    MeshDmp dmp;
    dmp.tau = 1.0;
    fmt::print("Learning...\n");
    dmp.embedding().setup(y0, g);
    const auto out = dmp.integrate(y0, g, ns_int, 1ms);

    std::vector<Eigen::Vector3d> res_traj(out.size());
    for (long i = 0; i < out.size(); ++i) {
        // rec.set_time_duration_secs("time", mdv::convert::seconds(out[i].t()));
        rec.set_time_sequence("tick", i);
        const Eigen::Vector3d pos = out[i].y().position();
        res_traj[i]               = pos;
        rec.log("meshdmp/trajectory/position", rr_converter(out[i].y()));
        rec.log("meshdmp/trajectory/x", rerun::Scalars(pos(0)));
        rec.log("meshdmp/trajectory/y", rerun::Scalars(pos(1)));
        rec.log("meshdmp/trajectory/z", rerun::Scalars(pos(2)));
    }
    rec.log_static("meshdmp/path", rr_converter(res_traj));


    using Pose = mdv::riemann::SE3::Point;
    std::vector<Pose> pos_demo;
    pos_demo.reserve(out.size());

    const auto project = [](const mdv::Vec3d& vec,
                            const mdv::Vec3d& normal) -> mdv::Vec3d {
        return (mdv::Mat3d::Identity() - normal * normal.transpose()) * vec;
    };

    const mdv::Vec3d x_des = mdv::Vec3d(0.0, 0.0, 0.8).normalized();

    const auto encode_rotation =
            [&x_des, &project](const mdv::mesh::Point& pt) -> Eigen::Quaterniond {
        const mdv::Vec3d z = pt.face().normal();
        const mdv::Vec3d x = project(x_des, z).normalized();
        const mdv::Vec3d y = z.cross(x);

        mdv::Mat3d mat;
        mat.col(0) = x;
        mat.col(1) = y;
        mat.col(2) = z;
        return Eigen::Quaterniond{mat};
    };

    for (const auto& sample : out) {
        pos_demo.emplace_back(sample.y().position(), encode_rotation(sample.y()));
    }

    for (std::size_t i = 0; i < pos_demo.size(); ++i) {
        rec.set_time_sequence("tick", i);
        rec.log("/meshdmp/pose", rr_converter(pos_demo[i].pos, pos_demo[i].ori, 20.0));
    }
    const auto se3_demo = Demo::builder()
                                  .assign_position(pos_demo)
                                  .velocity_automatic_differentiation()
                                  .acceleration_automatic_differentiation()
                                  .set_sampling_period(1ms)
                                  .create();
    Se3Dmp se3_dmp(100);
    se3_dmp.learn(se3_demo);
    se3_dmp.tau = 1.0;

    const auto se3_res =
            se3_dmp.integrate(pos_demo.front(), pos_demo.back(), se3_demo.size(), 1ms);

    for (long i = 0; i < se3_res.size(); ++i) {
        // rec.set_time_duration_secs("time", mdv::convert::seconds(out[i].t()));
        rec.set_time_sequence("tick", i);
        const auto& sample = se3_res[i].y();
        rec.log("/se3dmp/demonstration/x", rerun::Scalars(pos_demo[i].pos(0)));
        rec.log("/se3dmp/demonstration/y", rerun::Scalars(pos_demo[i].pos(1)));
        rec.log("/se3dmp/demonstration/z", rerun::Scalars(pos_demo[i].pos(2)));
        rec.log("/se3dmp/trajectory/x", rerun::Scalars(sample.pos(0)));
        rec.log("/se3dmp/trajectory/y", rerun::Scalars(sample.pos(1)));
        rec.log("/se3dmp/trajectory/z", rerun::Scalars(sample.pos(2)));
        rec.log("/se3dmp/pose", rr_converter(sample.pos, sample.ori, 20.0));
    }

    return 0;
}
