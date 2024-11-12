#include <string>

#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/rerun.hpp>
#include <mdv/config.hpp>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>

using mdv::mesh::Mesh;

int
main(int argc, char* argv[]) {
    std::string mesh_path = std::string(mdv::config::mesh_dir) + "/torus.off";
    if (argc > 1) mesh_path = std::string(argv[1]);

    const Mesh mesh(mesh_path);
    const auto rr_mesh = mdv::mesh::rerun::from_mesh(mesh);


    rerun::RecordingStream rec("mesh_visualiser");
    rec.spawn().exit_on_failure();
    rec.log(mesh.file_name(), rr_mesh);

    return 0;
}
