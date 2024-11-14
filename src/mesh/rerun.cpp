#include <algorithm>
#include <cstdint>
#include <spdlog/spdlog.h>

#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/rerun.hpp>
#include <mdv/utils/logging.hpp>
#include <rerun/archetypes/mesh3d.hpp>
#include <rerun/collection.hpp>
#include <rerun/components/position3d.hpp>
#include <rerun/components/triangle_indices.hpp>
#include <rerun/components/vector3d.hpp>

#include "mdv/mesh/fwd.hpp"

namespace rr  = rerun;
namespace rra = rerun::archetypes;
namespace rrc = rerun::components;

using mdv::mesh::Mesh;

namespace mdv::mesh::rerun::internal {

rrc::Vector3D        convert(const Eigen::Vector3d& x);
rrc::Position3D      vertex_to_position(const Mesh::Vertex& v);
rrc::TriangleIndices face_to_rr_triangle(const Mesh::Face& f);

std::vector<rrc::Position3D>      retrieve_vertices(const Mesh& mesh);
std::vector<rrc::Vector3D>        retrieve_vertex_normals(const Mesh& mesh);
std::vector<rrc::TriangleIndices> retrieve_triangles(const Mesh& mesh);

}  // namespace mdv::mesh::rerun::internal

mdv::SpdLoggerPtr rr_logger =
        mdv::static_logger_factory("rerun-mesh-adaptor", mdv::Debug);

rra::Mesh3D
mdv::mesh::rerun::from_mesh(const Mesh& mesh) {
    using ::rerun::components::Position3D;
    using ::rerun::components::TriangleIndices;
    using ::rerun::components::Vector3D;
    rr_logger->info("Converting mesh {} to Rerun Mesh3D", mesh.file_name());

    const auto vertices  = internal::retrieve_vertices(mesh);
    const auto triangles = internal::retrieve_triangles(mesh);
    const auto normals   = internal::retrieve_vertex_normals(mesh);

    rr_logger->debug("Returning Rerun Mesh3D");
    return rra::Mesh3D(vertices).with_triangle_indices(triangles).with_vertex_normals(
            normals
    );
}

//  ___       _                        _
// |_ _|_ __ | |_ ___ _ __ _ __   __ _| |___
//  | || '_ \| __/ _ \ '__| '_ \ / _` | / __|
//  | || | | | ||  __/ |  | | | | (_| | \__ \
// |___|_| |_|\__\___|_|  |_| |_|\__,_|_|___/
//

namespace mdv::mesh::rerun::internal {

rrc::Vector3D
convert(const Eigen::Vector3d& x) {
    return {static_cast<float>(x(0)), static_cast<float>(x(1)), static_cast<float>(x(2))
    };
}

rrc::Position3D
vertex_to_position(const Mesh::Vertex& v) {
    return {convert(v.position())};
}

rrc::TriangleIndices
face_to_rr_triangle(const Mesh::Face& f) {
    const auto [id1, id2, id3] = f.vertices_ids();
    return {static_cast<std::uint32_t>(id1),
            static_cast<std::uint32_t>(id2),
            static_cast<std::uint32_t>(id3)};
}

std::vector<rrc::Position3D>
retrieve_vertices(const Mesh& mesh) {
    rr_logger->debug("Converting vertices");
    std::vector<rrc::Position3D> vertices(mesh.num_vertices());
    std::transform(
            mesh.vertices_begin(),
            mesh.vertices_end(),
            vertices.begin(),
            internal::vertex_to_position
    );
    return vertices;
};

std::vector<rrc::TriangleIndices>
retrieve_triangles(const Mesh& mesh) {
    rr_logger->debug("Converting triangles");
    std::vector<rrc::TriangleIndices> triangles(mesh.num_faces());
    std::transform(
            mesh.faces_begin(),
            mesh.faces_end(),
            triangles.begin(),
            internal::face_to_rr_triangle
    );
    return triangles;
}

std::vector<rrc::Vector3D>
retrieve_vertex_normals(const Mesh& mesh) {
    rr_logger->debug("Converting vertex normals");
    std::vector<rrc::Vector3D> normals(mesh.num_vertices());
    std::transform(
            mesh.vertices_begin(),
            mesh.vertices_end(),
            normals.begin(),
            [](const Mesh::Vertex& v) -> rrc::Vector3D {
                return internal::convert(v.normal());
            }
    );
    return normals;
}

}  // namespace mdv::mesh::rerun::internal
