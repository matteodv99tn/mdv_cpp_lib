#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/base.h>
#include <iostream>
#include <sstream>
#include <string>

#include <range/v3/all.hpp>

#include "data.hpp"
#include "mdv/config.hpp"
#include "mdv/dmp/rhythmic_dmp.hpp"
#include "mdv/dmp/transformation_system/transformation_system.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/mesh_utilities.hpp"
#include "mdv/rerun.hpp"
#include "mdv/riemann_geometry/mesh.hpp"
#include "mdv/utils/conditions.hpp"

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_lines.hpp>
#include <rerun/archetypes/series_points.hpp>
#endif  // MDV_WITH_RERUN_SDK


namespace rs = ::ranges;
namespace rv = ::ranges::views;
using Quat   = Eigen::Quaterniond;

std::string
to_string(const Quat& q) {
    std::stringstream ss;
    ss << q;
    return ss.str();
}

using namespace mdv::mesh;
using std::filesystem::path;

template <typename T>
std::vector<double>
get_component_vector(const std::vector<T>& vec, const long id) {
    std::vector<double> res;
    res.reserve(vec.size());
    for (const auto& v : vec) res.emplace_back(v(id));
    return res;
}

Eigen::Vector3d
average(const mdv::mesh::Geodesic& path) {
    Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    return rs::accumulate(path, zero) / double(path.size());
}

Eigen::Vector3d
normal_projection(const mdv::mesh::Point& p, const Eigen::Vector3d& v) {
    const auto n = p.face().normal();
    return (mdv::Mat3d::Identity() - n * n.transpose()) * v;
}

void
write_matrix(
        FILE* file, const std::string& varname, const std::vector<Eigen::Vector3d>& data
) {
    fmt::println(file, "\n{} = np.array([", varname);
    for (const auto& v : data)
        fmt::println(file, "    [{}, {}, {}],", v(0), v(1), v(2));
    fmt::println(file, "])");
}

std::vector<Eigen::Vector3d>
move_outward(const mdv::mesh::Mesh& mesh, const std::vector<Eigen::Vector3d>& data) {
    std::vector<Eigen::Vector3d> res;
    res.reserve(data.size());

    for (const auto& p : data) {
        const auto pt = Point::from_cartesian(mesh, p);
        res.emplace_back(p + 0.003 * pt.face().normal());
    }
    return res;
}

int
main() {
    const long n_samples = 500;

    const auto eight_function = [](const double t) -> mdv::Vec2d {
        return {2 * std::cos(t), 0.9 * std::sin(2.0 * t + 0.1)};
    };

    auto* file = fopen("data.py", "w");
    fmt::println(file, "import numpy as np\n");

    // Rerun setup
    rerun::RecordingStream rec("meshdmp_comparison");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    const auto demo_3d = get_demonstration_position();

    using rerun::components::LineStrip2D, rerun::archetypes::LineStrips2D;
    auto path = LineStrip2D(
            demo_3d | rv::transform([](const auto& v) {
                return rerun::datatypes::Vec2D(v(0), v(1));
            })
            | rs::to_vector
    );
    rec.log_static("demonstration", LineStrips2D(std::move(path)));

    using namespace std::chrono_literals;

    using M    = mdv::riemann::MeshManifold;
    using Mesh = mdv::mesh::Mesh;
    using Demo = mdv::Demonstration<M>;

    const auto flat_mesh = Mesh::from_file(mdv::mesh::create_flat(5.0));
    const auto target_mesh =
            Mesh::from_file(mdv::config::meshes_directory() / "bunny_simple.off");

    const auto demo_position = demo_3d | rv::transform([&flat_mesh](const auto& p) {
                                   return Point::from_cartesian(flat_mesh, p);
                               })
                               | rs::to_vector;
    const Demo demo = Demo::builder()
                              .assign_position(demo_position)
                              .assign_velocity(get_demonstration_velocity())
                              .assign_acceleration(get_demonstration_acceleration())
                              .set_sampling_period(1ms)
                              .create();

    using Dmp = mdv::RhytmicDmp<M, mdv::riemann::MeshEmbedder>;

    const Point y0 = demo.front().y();
    const Point g  = Point::from_cartesian(flat_mesh, get_demonstration_centre());

    Dmp dmp(15, std::make_shared<mdv::dmp::TransformationSystem<M>>(20.0, 5.0));
    // Dmp dmp(15));
    dmp.tau = 1.0;
    fmt::print("Learning...\n");
    // dmp.embedding().setup_from_point_and_direction(
    //         demo.front().y(), Eigen::Vector3d::UnitX()
    // );
    dmp.embedding().setup(
            demo.front().y(),
            Point::from_cartesian(flat_mesh, get_demonstration_centre())
    );
    dmp.learn(demo, 1.0, g);
    fmt::print("Learning... Done!\n");

    using rerun::components::LineStrip3D, rerun::archetypes::LineStrips3D;
    const auto tmp           = get_integrated_position();
    auto       original_path = LineStrip3D(
            tmp | rv::transform([](const auto& p) {
                return rerun::datatypes::Vec3D(p(0), p(1), p(2));
            })
            | rs::to_vector
    );
    rec.log_static("bunny/mesh", rr_converter(target_mesh));
    rec.log_static("bunny/original_path", LineStrips3D(std::move(original_path)));

    const auto       new_g = Point::from_cartesian(target_mesh, get_bunny_centre());
    const auto       n     = new_g.face().normal();
    const mdv::Vec3d dir   = normal_projection(new_g, mdv::Vec3d::UnitY()).normalized();
    assert(mdv::condition::are_orthogonal(dir, n));

    const auto new_y0 =
            Point::from_cartesian(target_mesh, get_integrated_position()[0]);
    const mdv::Vec3d v0 = get_integrated_velocity()[0];
    dmp.embedding().setup(new_y0, new_g);

    // const auto new_path = dmp.integrate(new_y0, v0, new_g, 0.28, 3 * n_samples, 1ms);
    const auto new_path = dmp.integrate(new_y0, v0, new_g, 0.28, n_samples, 3ms);

    auto bunny_path = LineStrip3D(
            new_path | rv::transform([](const auto& sample) {
                const auto p = sample.y().position();
                return rerun::datatypes::Vec3D(p(0), p(1), p(2));
            })
            | rs::to_vector
    );
    rec.log_static("bunny/path", LineStrips3D(std::move(bunny_path)));


    const auto new_position =
            new_path | rv::transform([](const auto& s) { return s.y().position(); })
            | rs::to_vector;

    write_matrix(
            file,
            "OLD_MESHDMP_PATH",
            move_outward(target_mesh, get_integrated_position())
    );
    write_matrix(file, "NEW_MESHDMP_PATH", move_outward(target_mesh, new_position));
    fclose(file);
}
