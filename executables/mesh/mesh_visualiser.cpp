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
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/tangent_vector.hpp"

using mdv::mesh::Mesh;

int
main(int argc, char* argv[]) {
    std::srand(std::time(nullptr));
    std::string mesh_path = std::string(mdv::config::mesh_dir) + "/torus_simple.off";
    if (argc > 1) mesh_path = std::string(argv[1]);
    std::cout << "Selected mesh path: " << mesh_path << '\n';

    const auto mesh = Mesh::from_file(mesh_path);
    std::cout << "Mesh loaded\n";

    mdv::RerunConverter to_rerun;

    rerun::RecordingStream rec("mesh_visualiser");
    rec.spawn().exit_on_failure();

    std::cout << "Picking random points\n";
    const auto pt1 = Mesh::Point::random(mesh);
    const auto pt2 = Mesh::Point::random(mesh);

    rec.log("points", to_rerun(std::vector({pt1, pt2})));
    rec.log_static("mesh", to_rerun(mesh));

    std::cout << "Computing the geodesic between the points\n";
    const auto geodesic = mesh.build_geodesic(pt1, pt2);
    if (geodesic.empty()) {
        std::cout << "Geodesic is empty! Terminating\n";
        return 1;
    }

    auto rr_geod = to_rerun(geodesic);
    rec.log("geodesic", rerun::archetypes::LineStrips3D({rr_geod}));

    const auto tv1a = mdv::mesh::logarithmic_map(pt1, pt2);
    const auto tv1b = mdv::mesh::TangentVector::unit_random(pt1);
    const auto tv2a = mdv::mesh::parallel_transport(tv1a, pt2);
    const auto tv2b = mdv::mesh::parallel_transport(tv1b, pt2);

    rec.log("logarithmic_map", to_rerun(std::vector({tv1a, tv2a})));
    rec.log("tangent_vectors", to_rerun(std::vector({tv1b, tv2b})));

    return 0;
}
