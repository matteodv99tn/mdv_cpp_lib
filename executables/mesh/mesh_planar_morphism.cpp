#include <fmt/os.h>
#include <string>

#include "mdv/config.hpp"
#include "mdv/mesh/flat_parameterisation.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/spdlog.hpp"

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/line_strips3d.hpp>
#include <rerun/recording_stream.hpp>

#include "mdv/rerun.hpp"
#endif  // MDV_WITH_RERUN_SDK


using namespace mdv::mesh;

int
main(int argc, char* argv[]) {
    Mesh::default_logger = mdv::static_logger_factory("Mesh");
    Mesh::default_logger->set_log_level(mdv::Logger::LogLevel::Debug);

    const std::string mesh_path =
            mdv::config::meshes_directory() / "fender_low_res.stl";

    auto        mesh     = Mesh::from_file(mesh_path);
    const Point p0       = mesh.vertex(3884);  // NOLINT: extracted from meshlab
    auto        sub_mesh = Mesh::extract_normal_bounded_surface(mesh, p0, 40.0);

    FlatParameterisation param(sub_mesh);

    std::vector<Eigen::Vector2d> projs;
    projs.reserve(sub_mesh.num_vertices());
    for (long i = 0; i < sub_mesh.num_vertices(); ++i)
        projs.emplace_back(param.project(sub_mesh.vertex(i)));

    std::vector<Eigen::Vector3d> reconstructed;
    reconstructed.reserve(sub_mesh.num_vertices());
    for (const auto& uv : projs)
        reconstructed.emplace_back(param.retrieve(uv).position());

#ifdef MDV_WITH_RERUN_SDK
    mdv::RerunConverter    to_rerun;
    rerun::RecordingStream rec("mesh_extraction");
    rec.spawn().exit_on_failure();

    rec.log_static("mesh", to_rerun(mesh));
    rec.log_static("sub_mesh", to_rerun(sub_mesh));


    std::vector<rerun::components::Position3D> planar_pos;
    planar_pos.reserve(sub_mesh.num_vertices());
    for (long i = 0; i < sub_mesh.num_vertices(); ++i) {
        planar_pos.emplace_back(projs[i](0), projs[i](1), 0.0);
    }

    std::vector<rerun::components::Position3D> reconstructed_pos;
    reconstructed_pos.reserve(sub_mesh.num_vertices());
    for (long i = 0; i < sub_mesh.num_vertices(); ++i) {
        reconstructed_pos.emplace_back(
                reconstructed[i](0), reconstructed[i](1), reconstructed[i](2)
        );
    }

    rec.log_static(
            "uv_vertices", rerun::Points3D().with_positions(std::move(planar_pos))
    );
    rec.log_static(
            "reconstructed_vertices",
            rerun::Points3D().with_positions(std::move(reconstructed_pos))
    );
#endif  // MDV_WITH_RERUN_SDK

    constexpr double err_th  = 1e-9;
    long             n_wrong = 0;
    for (long i = 0; i < sub_mesh.num_vertices(); ++i) {
        const auto v   = sub_mesh.vertex(i).position();
        const auto err = (v - reconstructed[i]).norm();
        if (err > err_th) ++n_wrong;
    }

    fmt::println(
            "Number of wrongly reconstructed vertices: {} / {}",
            n_wrong,
            sub_mesh.num_vertices()
    );

    return 0;
}
