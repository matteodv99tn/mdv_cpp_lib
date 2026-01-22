#include <fmt/os.h>
#include <string>

#include "mdv/config.hpp"
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

    const auto  mesh     = Mesh::from_file(mesh_path);
    const Point p0       = mesh.vertex(3884);  // NOLINT: extracted from meshlab
    const auto  sub_mesh = Mesh::extract_normal_bounded_surface(mesh, p0, 40.0);

    FlatParameterisation param(sub_mesh);

#ifdef MDV_WITH_RERUN_SDK
    mdv::RerunConverter    to_rerun;
    rerun::RecordingStream rec("mesh_extraction");
    rec.spawn().exit_on_failure();

    rec.log_static("mesh", to_rerun(mesh));
    rec.log_static("sub_mesh", to_rerun(sub_mesh));
#endif  // MDV_WITH_RERUN_SDK
    return 0;
}
