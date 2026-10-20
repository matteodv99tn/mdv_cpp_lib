#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <gsl/pointers>
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


namespace {
using namespace mdv::mesh;
namespace rs = ::ranges;
namespace rv = ::ranges::views;
using Quat   = Eigen::Quaterniond;

std::string
to_string(const Quat& q) {
    std::stringstream ss;
    ss << q;
    return ss.str();
}

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
normal_projection(const Mesh& mesh, const Point& p, const Eigen::Vector3d& v) {
    const auto n = normal(mesh, face(mesh, p));
    return (mdv::Mat3d::Identity() - n * n.transpose()) * v;
}

void
write_matrix(
        FILE* file, const std::string& varname, const std::vector<Eigen::Vector3d>& data
) {
    fmt::print(file, "\n{} = np.array([\n", varname);
    for (const auto& v : data)
        fmt::print(file, "    [{}, {}, {}],\n", v(0), v(1), v(2));
    fmt::print(file, "])\n");
}

std::vector<Eigen::Vector3d>
move_outward(const mdv::mesh::Mesh& mesh, const std::vector<Eigen::Vector3d>& data) {
    std::vector<Eigen::Vector3d> res;
    res.reserve(data.size());

    for (const auto& p : data) {
        const auto pt = from_cartesian(mesh, p);
        res.emplace_back(p + 0.003 * normal(mesh, face(mesh, pt)));
    }
    return res;
}

}  // namespace

int
main() {
    using namespace mdv::mesh;
    const long n_samples = 500;

    const auto eight_function = [](const double t) -> mdv::Vec2d {
        return {2 * std::cos(t), 0.9 * std::sin(2.0 * t + 0.1)};
    };

    gsl::owner<FILE*> file = fopen("data.py", "w");
    fmt::print(file, "import numpy as np\n\n");

    // Rerun setup
    rerun::RecordingStream rec("meshdmp_comparison");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    const auto demo_3d = get_demonstration_position();

    using rerun::components::LineStrip2D, rerun::archetypes::LineStrips2D;
    auto path = LineStrip2D(
            demo_3d | rv::transform([](const auto& v) {
                return rerun::datatypes::Vec2D(
                        static_cast<float>(v(0)), static_cast<float>(v(1))
                );
            })
            | rs::to_vector
    );
    rec.log_static("demonstration", LineStrips2D(std::move(path)));

    using namespace std::chrono_literals;

    using M    = mdv::riemann::MeshManifold;
    using Mesh = mdv::mesh::Mesh;
    using Demo = mdv::Demonstration<M>;

    const auto flat_owner =
            std::make_shared<Mesh>(Mesh::from_file(mdv::mesh::create_flat(5.0)));
    const auto target_owner = std::make_shared<Mesh>(
            Mesh::from_file(mdv::config::meshes_directory() / "bunny_simple.off")
    );
    const auto& flat_mesh   = *flat_owner;
    const auto& target_mesh = *target_owner;
    auto        m           = std::make_shared<M>(flat_owner);

    const auto demo_position = demo_3d | rv::transform([&flat_mesh](const auto& p) {
                                   return from_cartesian(flat_mesh, p);
                               })
                               | rs::to_vector;
    const Demo demo = Demo::builder(m)
                              .assign_position(demo_position)
                              .assign_velocity(get_demonstration_velocity())
                              .assign_acceleration(get_demonstration_acceleration())
                              .set_sampling_period(1ms)
                              .create();

    using Dmp = mdv::RhytmicDmp<M, mdv::riemann::MeshEmbedder>;

    const Point y0 = demo.front().y();
    const Point g  = from_cartesian(flat_mesh, get_demonstration_centre());

    auto ts = std::make_shared<Dmp::TransformationSystem>(20.0, 5.0, m);
    Dmp  dmp(15, ts, mdv::get_default_logger(), m);
    // Dmp dmp(15));
    dmp.tau = 1.0;
    fmt::print("Learning...\n");
    // dmp.embedding().setup_from_point_and_direction(
    //         demo.front().y(), Eigen::Vector3d::UnitX()
    // );
    dmp.embedding().setup(
            demo.front().y(), from_cartesian(flat_mesh, get_demonstration_centre())
    );
    dmp.learn(demo, 1.0, g);
    fmt::print("Learning... Done!\n");

    using rerun::components::LineStrip3D, rerun::archetypes::LineStrips3D;
    const auto tmp           = get_integrated_position();
    auto       original_path = LineStrip3D(
            tmp | rv::transform([](const auto& p) {
                return rerun::datatypes::Vec3D(
                        static_cast<float>(p(0)),
                        static_cast<float>(p(1)),
                        static_cast<float>(p(2))
                );
            })
            | rs::to_vector
    );
    rec.log_static("bunny/mesh", rr_converter(target_mesh));
    rec.log_static("bunny/original_path", LineStrips3D(std::move(original_path)));

    const auto       new_g = from_cartesian(target_mesh, get_bunny_centre());
    const auto       n     = normal(target_mesh, face(target_mesh, new_g));
    const mdv::Vec3d dir =
            normal_projection(target_mesh, new_g, mdv::Vec3d::UnitY()).normalized();
    assert(mdv::condition::are_orthogonal(dir, n));

    const auto       new_y0 = from_cartesian(target_mesh, get_integrated_position()[0]);
    const mdv::Vec3d v0     = get_integrated_velocity()[0];
    // Preserve the shared manifold identity held by the TS and embedder.
    *m = M(target_owner);
    dmp.embedding().setup(new_y0, new_g);

    // const auto new_path = dmp.integrate(new_y0, v0, new_g, 0.28, 3 * n_samples, 1ms);
    const auto new_path = dmp.integrate(new_y0, v0, new_g, 0.28, n_samples, 3ms);

    auto bunny_path = LineStrip3D(
            new_path | rv::transform([&target_mesh](const auto& sample) {
                const auto p = position(target_mesh, sample.y());
                return rerun::datatypes::Vec3D(
                        static_cast<float>(p(0)),
                        static_cast<float>(p(1)),
                        static_cast<float>(p(2))
                );
            })
            | rs::to_vector
    );
    rec.log_static("bunny/path", LineStrips3D(std::move(bunny_path)));


    const auto new_position = new_path | rv::transform([&target_mesh](const auto& s) {
                                  return position(target_mesh, s.y());
                              })
                              | rs::to_vector;

    write_matrix(
            file,
            "OLD_MESHDMP_PATH",
            move_outward(target_mesh, get_integrated_position())
    );
    write_matrix(file, "NEW_MESHDMP_PATH", move_outward(target_mesh, new_position));
    fclose(file);
}
