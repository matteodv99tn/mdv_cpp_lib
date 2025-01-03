#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

#include <mdv/config.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/rerun.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/line_strips3d.hpp>
#include <rerun/recording_stream.hpp>

using mdv::mesh::Mesh;

int
main(int argc, char* argv[]) {
    std::srand(std::time(nullptr));
    std::string mesh_path = std::string(mdv::config::mesh_dir) + "/torus.off";
    if (argc > 1) mesh_path = std::string(argv[1]);
    std::cout << "Selected mesh path: " << mesh_path << std::endl;

    const auto mesh = Mesh::from_file(mesh_path);
    std::cout << "Mesh loaded\n";
    const auto rr_mesh = mdv::rerun_convert::mesh(mesh);

    rerun::RecordingStream rec("mesh_visualiser");
    rec.spawn().exit_on_failure();

    std::cout << "Picking random points\n";
    const auto pt1 = Mesh::Point::random(mesh);
    const auto pt2 = Mesh::Point::random(mesh);

    rec.log("points", mdv::rerun_convert::points({pt1, pt2}));
    rec.log_static("mesh", rr_mesh);

    std::cout << "Computing the geodesic between the points\n";
    const auto geodesic = mesh.build_geodesic(pt1, pt2);
    if (geodesic.empty()) return 1;
    auto rr_geod = mdv::rerun_convert::geodesic(geodesic);
    rec.log("geodesic", rerun::archetypes::LineStrips3D({rr_geod}));

    return 0;
}
