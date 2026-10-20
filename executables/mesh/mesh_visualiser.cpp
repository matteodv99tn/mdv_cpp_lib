#include <cstdlib>
#include <fmt/os.h>
#include <iostream>
#include <string>

#include "mdv/mesh/mesh_manager.hpp"
#include "mdv/mesh/mesh_utilities.hpp"
#include "mdv/utils/spdlog.hpp"

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
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/rerun.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

int
main(int argc, char* argv[]) {
    using namespace mdv::mesh;
#ifdef MDV_WITH_RERUN_SDK
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    // std::string mesh_path = mdv::config::mesh_dir() + "/torus_simple.off";
    // // mesh_path             = mdv::mesh::create_cone(6);
    // if (argc > 1) mesh_path = std::string(argv[1]);
    // std::cout << "Selected mesh path: " << mesh_path << '\n';
    //
    const std::string mesh_path = "mesh.off";
    mdv::mesh::create_from_function(mesh_path, [](double x, double y) -> double {
        return x * x + y * y;
    });

    MeshManager::default_logger = mdv::static_logger_factory("MeshManager");
    Mesh::default_logger        = mdv::static_logger_factory("Mesh");
    Mesh::default_logger->set_log_level(mdv::Logger::LogLevel::Trace);
    MeshManager::default_logger->set_log_level(mdv::Logger::LogLevel::Trace);
    // Mesh::default_logger->set_log_level(mdv::Logger::LogLevel::Debug);
    MeshManager mm;

    const auto  mesh_owner  = mm.get_mesh_from_file(mesh_path);
    const auto  mesh2_owner = mm.get_mesh_from_file(mesh_path);
    const auto& mesh        = *mesh_owner;
    const auto& mesh2       = *mesh2_owner;

    // const auto mesh = Mesh::from_file(mesh_path);
    std::cout << "Mesh loaded\n";

    mdv::RerunConverter to_rerun;

    rerun::RecordingStream rec("mesh_visualiser");
    rec.spawn().exit_on_failure();

    rec.log_static("mesh", to_rerun(mesh));
    // return 0;

    std::cout << "Picking random points\n";
    const auto pt1 = random_point(mesh);
    const auto pt2 = random_point(mesh);

    rec.log("points", to_rerun(mesh, std::vector({pt1, pt2})));

    std::cout << "Computing the geodesic between the points\n";
    const auto geodesic = build_geodesic(mesh, pt1, pt2);
    if (geodesic.empty()) {
        std::cout << "Geodesic is empty! Terminating\n";
        return 1;
    }

    assert(!geodesic.empty());
    rec.log("geodesic", to_rerun(geodesic));

    const auto log  = logarithmic_map(mesh, pt1, pt2);
    const auto tv1a = logarithmic_map(mesh, pt1, pt2).normalised();
    const auto tv1b = TangentVector::unit_random(mesh, pt1);
    const auto tv2a = parallel_transport(mesh, tv1a, pt2);
    const auto tv2b = parallel_transport(mesh, tv1b, pt2);

    std::cout << "Computing exponential map\n";
    mdv::mesh::Geodesic exp_geodesic;
    const auto          pt2computed = exponential_map(mesh, log, &exp_geodesic);

    rec.log("logarithmic_map", to_rerun(mesh, std::vector({tv1a, tv2a})));
    rec.log("tangent_vectors", to_rerun(mesh, std::vector({tv1b, tv2b})));
    rec.log("exponential_map", to_rerun(exp_geodesic));

#endif  // MDV_WITH_RERUN_SDK
    return 0;
}
