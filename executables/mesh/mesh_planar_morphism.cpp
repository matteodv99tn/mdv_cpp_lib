#include <fmt/format.h>
#include <fmt/os.h>
#include <limits>
#include <stdexcept>
#include <string>

#include "mdv/config.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/flat_parameterisation.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/sampler.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"
#include "mdv/utils/spdlog.hpp"

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/line_strips3d.hpp>
#include <rerun/recording_stream.hpp>

#include "mdv/rerun.hpp"
#endif  // MDV_WITH_RERUN_SDK


int
main(int argc, char* argv[]) {
    using namespace mdv::mesh;
    Mesh::default_logger = mdv::static_logger_factory("Mesh");
    Mesh::default_logger->set_log_level(mdv::Logger::LogLevel::Debug);

    // const std::string mesh_path =
    //         mdv::config::meshes_directory() / "fender_low_res.stl";
    // auto        mesh     = Mesh::from_file(mesh_path);
    // const Point p0 = PointOnVertex{VertexId{3884}}; // extracted from meshlab
    // auto        sub_mesh = Mesh::extract_normal_bounded_surface(mesh, p0, 20.0);

    const std::string mesh_path = mdv::config::meshes_directory() / "bunny_simple.off";
    auto              mesh      = Mesh::from_file(mesh_path);
    const Point       p0        = PointOnVertex{VertexId{0}};  // extracted from meshlab
    auto              sub_mesh  = Mesh::extract_normal_bounded_surface(mesh, p0, 50.0);
    auto              filled_mesh = Mesh::fill_holes(sub_mesh);


    FlatParameterisation param(filled_mesh);

    std::vector<Eigen::Vector2d> projs;
    projs.reserve(sub_mesh.num_vertices());
    for (mdv::mesh::Index i = 0; i < sub_mesh.num_vertices(); ++i)
        projs.emplace_back(param.project(PointOnVertex{VertexId{i}}));

    std::vector<Eigen::Vector3d> reconstructed;
    reconstructed.reserve(sub_mesh.num_vertices());
    for (const auto& uv : projs)
        reconstructed.emplace_back(position(filled_mesh, param.retrieve(uv)));

#ifdef MDV_WITH_RERUN_SDK
    mdv::RerunConverter    to_rerun;
    rerun::RecordingStream rec("mesh_extraction");
    rec.spawn().exit_on_failure();

    rec.log_static("mesh", to_rerun(mesh));
    rec.log_static("sub_mesh", to_rerun(sub_mesh));
    rec.log_static("filled_mesh", to_rerun(filled_mesh));


    std::vector<rerun::components::Position3D> planar_pos;
    planar_pos.reserve(sub_mesh.num_vertices());
    for (mdv::mesh::Index i = 0; i < sub_mesh.num_vertices(); ++i) {
        planar_pos.emplace_back(projs[i](0), projs[i](1), 0.0);
    }

    std::vector<rerun::components::Position3D> reconstructed_pos;
    reconstructed_pos.reserve(sub_mesh.num_vertices());
    for (mdv::mesh::Index i = 0; i < sub_mesh.num_vertices(); ++i) {
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
    for (mdv::mesh::Index i = 0; i < sub_mesh.num_vertices(); ++i) {
        const auto v   = position(sub_mesh, mdv::mesh::VertexId{i});
        const auto err = (v - reconstructed[i]).norm();
        if (err > err_th) ++n_wrong;
    }

    fmt::print(
            "Number of wrongly reconstructed vertices: {} / {}\n",
            n_wrong,
            sub_mesh.num_vertices()
    );

    double             max_err = 0.0;
    mdv::mesh::Sampler sampler(sub_mesh, 42);
    for (long i = 0; i < 2000; ++i) {
        const auto   pt  = sampler.point_in_face();
        const auto   uv  = param.project(pt);
        const auto   rev = param.retrieve(uv);
        const double e   = (position(sub_mesh, pt) - position(filled_mesh, rev)).norm();
        max_err          = std::max(max_err, e);
    }
    fmt::print("Maximum reconstruction error: {}\n", max_err);

    max_err            = 0.0;
    constexpr double b = 1.0 / 3.0;
    for (mdv::mesh::Index i = 0; i < sub_mesh.num_faces(); ++i) {
        const Point pt = PointInFace{
                .f = FaceId{i},
                  .bary = {b, b, b}
        };
        const auto   uv  = param.project(pt);
        const auto   rev = param.retrieve(uv);
        const double e   = (position(sub_mesh, pt) - position(filled_mesh, rev)).norm();
        max_err          = std::max(max_err, e);
        if (e > 1e-6) { fmt::print("Err at face {} = {}\n", i, e); }
    }
    fmt::print("Maximum reconstruction error: {}\n", max_err);


    fmt::print("Checking hole filling algorithm result\n");
    if (filled_mesh.num_faces() < sub_mesh.num_faces())
        throw std::runtime_error("Filled mesh has less face than starting input!");

    if (filled_mesh.num_vertices() != sub_mesh.num_vertices()) {
        throw std::runtime_error(
                "Filled mesh has a different number of vertices then  starting input!"
        );
    }

    for (mdv::mesh::Index i = 0; i < sub_mesh.num_vertices(); ++i) {
        using mdv::condition::are_equal;
        if (!are_equal(
                    position(sub_mesh, mdv::mesh::VertexId{i}),
                    position(filled_mesh, mdv::mesh::VertexId{i})
            ))
            throw std::runtime_error("Vertices have been moved");
    }

    for (mdv::mesh::Index i = 0; i < sub_mesh.num_faces(); ++i) {
        const auto v1s = vertices(sub_mesh, FaceId{i});
        const auto v2s = vertices(filled_mesh, FaceId{i});

        if ((v1s[0] != v2s[0]) || (v1s[1] != v2s[1]) || (v1s[2] != v2s[2])) {
            throw std::runtime_error("Faces have been changed");
        }
    }
    fmt::print("Checking hole filling algorithm result -- Passed!\n");

    return 0;
}
