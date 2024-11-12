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

namespace rra = rerun::archetypes;

using mdv::mesh::Mesh;

mdv::SpdLoggerPtr rr_logger =
        mdv::static_logger_factory("rerun-mesh-adaptor", mdv::Debug);

rra::Mesh3D
mdv::mesh::rerun::from_mesh(const Mesh& mesh) {
    using ::rerun::components::Position3D;
    using ::rerun::components::TriangleIndices;
    rr_logger->info("Converting mesh {} to Rerun Mesh3D", mesh.file_name());

    rr_logger->debug("Converting vertices");
    std::vector<Position3D> vertices(mesh.num_vertices());
    std::transform(
            mesh.vertices_begin(),
            mesh.vertices_end(),
            vertices.begin(),
            [](const Mesh::Vertex& v) -> Position3D {
                auto to_float   = [](const auto& d) { return static_cast<float>(d); };
                const auto& pos = v.position();
                return {to_float(pos(0)), to_float(pos(1)), to_float(pos(2))};
            }
    );
    rr_logger->debug("Converting triangles");
    std::vector<TriangleIndices> triangles(mesh.num_faces());
    std::transform(
            mesh.faces_begin(),
            mesh.faces_end(),
            triangles.begin(),
            [](const Mesh::Face& f) -> TriangleIndices {
                const auto [id1, id2, id3] = f.vertices_ids();
                return {static_cast<std::uint32_t>(id1),
                        static_cast<std::uint32_t>(id2),
                        static_cast<std::uint32_t>(id3)};
            }
    );
    rr_logger->debug("Returning Rerun Mesh3D");
    return rra::Mesh3D(vertices).with_triangle_indices(triangles);
}
