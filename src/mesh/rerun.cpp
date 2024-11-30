#include <algorithm>
#include <cstdint>
#include <spdlog/spdlog.h>
#include <stdexcept>

#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/rerun.hpp>
#include <mdv/utils/logging.hpp>
#include <mdv/utils/logging_extras.hpp>
#include <rerun/archetypes/mesh3d.hpp>
#include <rerun/archetypes/points3d.hpp>
#include <rerun/collection.hpp>
#include <rerun/components/position3d.hpp>
#include <rerun/components/triangle_indices.hpp>
#include <rerun/components/vector3d.hpp>

#include "mdv/mesh/fwd.hpp"

namespace rr  = rerun;
namespace rra = rerun::archetypes;
namespace rrc = rerun::components;
namespace rrd = rerun::datatypes;

using mdv::mesh::Mesh;

namespace mdv::mesh::rerun_convert::internal {

rrd::Vec3D           convert(const Eigen::Vector3d& x);
rrc::Position3D      vertex_to_position(const Mesh::Vertex& v);
rrc::Position3D      position(const Mesh::Point& pt);
rrc::TriangleIndices face_to_rr_triangle(const Mesh::Face& f);

std::vector<rrc::Position3D>      retrieve_vertices(const Mesh& mesh);
std::vector<rrc::Vector3D>        retrieve_vertex_normals(const Mesh& mesh);
std::vector<rrc::TriangleIndices> retrieve_triangles(const Mesh& mesh);

}  // namespace mdv::mesh::rerun_convert::internal

static mdv::SpdLoggerPtr rr_logger =
        mdv::static_logger_factory("rerun-mesh-adaptor", mdv::Debug);

rra::Mesh3D
mdv::mesh::rerun_convert::mesh(const Mesh& mesh) {
    using ::rerun::components::Position3D;
    using ::rerun::components::TriangleIndices;
    using ::rerun::components::Vector3D;
    rr_logger->info("Converting mesh {} to Rerun Mesh3D", mesh.name());

    const auto vertices  = internal::retrieve_vertices(mesh);
    const auto triangles = internal::retrieve_triangles(mesh);
    const auto normals   = internal::retrieve_vertex_normals(mesh);

    if (normals.size() != vertices.size())
        rr_logger->warn("Vertex normals count differs from the number of vertices");
    if (vertices.size() != mesh.num_vertices())
        rr_logger->warn("Rerun and mdv::Mesh mismatch in vertex count");

    rr_logger->debug("Returning Rerun Mesh3D");
    return rra::Mesh3D(vertices).with_triangle_indices(triangles).with_vertex_normals(
            normals
    );
}

rrc::LineStrip3D
mdv::mesh::rerun_convert::geodesic(const mdv::mesh::Geodesic& geod) {
    rr_logger->info("Exporting geodesic line");
    if (geod.empty()) {
        rr_logger->warn("Geodesic line has no points!");
        throw std::runtime_error("Not enough points");
    }
    std::vector<rrc::Position3D> points(geod.size());
    std::transform(geod.begin(), geod.end(), points.begin(), internal::convert);
    return {points};
}

rra::Arrows3D
mdv::mesh::rerun_convert::vertex_normals(const Mesh& mesh) {
    std::vector<rrc::Vector3D> normals(mesh.num_vertices());
    std::transform(
            mesh.vertices_begin(),
            mesh.vertices_end(),
            normals.begin(),
            [](const Mesh::Vertex& v) -> rrc::Vector3D {
                return internal::convert(v.normal());
            }
    );
    const auto vertices = internal::retrieve_vertices(mesh);
    return rra::Arrows3D::from_vectors(normals).with_origins(vertices);
}

rra::Points3D
mdv::mesh::rerun_convert::point(const Mesh::Point& pt) {
    return rra::Points3D({internal::position(pt)});
}

rra::Points3D
mdv::mesh::rerun_convert::points(const std::vector<Mesh::Point>& pts) {
    std::vector<rrc::Position3D> positions(pts.size());
    std::transform(pts.begin(), pts.end(), positions.begin(), internal::position);
    return rra::Points3D(std::move(positions));
}

//  ___       _                        _
// |_ _|_ __ | |_ ___ _ __ _ __   __ _| |___
//  | || '_ \| __/ _ \ '__| '_ \ / _` | / __|
//  | || | | | ||  __/ |  | | | | (_| | \__ \
// |___|_| |_|\__\___|_|  |_| |_|\__,_|_|___/
//

namespace mdv::mesh::rerun_convert::internal {

rrd::Vec3D
convert(const Eigen::Vector3d& x) {
    return {static_cast<float>(x(0)), static_cast<float>(x(1)), static_cast<float>(x(2))
    };
}

rrc::Position3D
vertex_to_position(const Mesh::Vertex& v) {
    return {convert(v.position())};
}

rrc::Position3D
position(const Mesh::Point& pt) {
    rr_logger->debug("Exporting point at position {}", eigen_to_str(pt.position()));
    return convert(pt.position());
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

}  // namespace mdv::mesh::rerun_convert::internal
