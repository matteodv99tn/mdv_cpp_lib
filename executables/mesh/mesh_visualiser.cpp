#include <cstdlib>
#include <fmt/os.h>
#include <iostream>
#include <string>

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/line_strips3d.hpp>
#include <rerun/recording_stream.hpp>
#endif  // MDV_WITH_RERUN_SDK

#include "mdv/config.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/rerun.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

using mdv::mesh::Mesh;

int
main(int argc, char* argv[]) {
#ifdef MDV_WITH_RERUN_SDK
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

    rec.log("geodesic", to_rerun(geodesic));

    const auto log  = mdv::mesh::logarithmic_map(pt1, pt2);
    const auto tv1a = mdv::mesh::logarithmic_map(pt1, pt2).normalised();
    const auto tv1b = mdv::mesh::TangentVector::unit_random(pt1);
    const auto tv2a = mdv::mesh::parallel_transport(tv1a, pt2);
    const auto tv2b = mdv::mesh::parallel_transport(tv1b, pt2);

    mdv::mesh::Geodesic exp_geodesic;
    const auto          pt2computed = mdv::mesh::exponential_map(log, &exp_geodesic);

    rec.log("logarithmic_map", to_rerun(std::vector({tv1a, tv2a})));
    rec.log("tangent_vectors", to_rerun(std::vector({tv1b, tv2b})));
    rec.log("exponential_map", to_rerun(exp_geodesic));

#endif  // MDV_WITH_RERUN_SDK
    return 0;
}
