#include <algorithm>
#include <cstdint>
#include <iterator>
#include <spdlog/spdlog.h>
#include <stdexcept>

#include <mdv/mesh/mesh.hpp>
#include <mdv/rerun.hpp>
#include <mdv/utils/logging.hpp>
#include <mdv/utils/logging_extras.hpp>
#include <rerun/archetypes/mesh3d.hpp>
#include <rerun/archetypes/points3d.hpp>
#include <rerun/collection.hpp>
#include <rerun/components/position3d.hpp>
#include <rerun/components/triangle_indices.hpp>
#include <rerun/components/vector3d.hpp>

#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/tangent_vector.hpp"

namespace rra = rerun::archetypes;
namespace rrc = rerun::components;
namespace rrd = rerun::datatypes;

using mdv::RerunConverter;
using mdv::mesh::Mesh;
using mdv::mesh::TangentVector;

mdv::SpdLoggerPtr RerunConverter::_logger =
        mdv::static_logger_factory("RerunConverter");

//  ____        _     _ _
// |  _ \ _   _| |__ | (_) ___
// | |_) | | | | '_ \| | |/ __|
// |  __/| |_| | |_) | | | (__
// |_|    \__,_|_.__/|_|_|\___|
//

rra::Mesh3D
RerunConverter::operator()(const Mesh& mesh) const {
    _logger->info("Converting mesh {} to Rerun Mesh3D", mesh.name());
    auto vertices  = mesh_vertices(mesh);
    auto triangles = mesh_triangles(mesh);
    auto normals   = mesh_vertex_normals(mesh);

    if (normals.size() != vertices.size())
        _logger->warn("Vertex normals count differs from the number of vertices");
    if (vertices.size() != mesh.num_vertices())
        _logger->warn("Rerun and mdv::Mesh mismatch in vertex count");
    if (triangles.size() != mesh.num_faces())
        _logger->warn("Rerun and mdv::Mesh mismatch in face count");

    _logger->info("triangles size: {}", triangles.size());

    return rra::Mesh3D(std::move(vertices))
            .with_triangle_indices(std::move(triangles))
            .with_vertex_normals(std::move(normals));
}

rrc::LineStrip3D
RerunConverter::operator()(const mdv::mesh::Geodesic& geod) const {
    _logger->info("Exporting geodesic line");
    if (geod.empty()) {
        _logger->warn("Geodesic line has no points!");
        throw std::runtime_error("Not enough points");
    }
    std::vector<rrc::Position3D> points(geod.size());
    std::transform(geod.begin(), geod.end(), points.begin(), [this](const auto& pt) {
        return (*this)(pt);
    });
    return {points};
}

rra::Points3D
RerunConverter::operator()(const Mesh::Point& pt) const {
    _logger->info(
            "Exporting points on a mesh at position {}", eigen_to_str(pt.position())
    );
    return rra::Points3D((*this)(pt.position()));
}

rra::Points3D
RerunConverter::operator()(const std::vector<Mesh::Point>& pts) const {
    _logger->info("Exporting {} points on a mesh", pts.size());

    std::vector<rrc::Position3D> positions;
    positions.reserve(pts.size());
    for (const auto& p : pts) positions.emplace_back(operator()(p.position()));

    return rra::Points3D(std::move(positions));
}

rra::Arrows3D
RerunConverter::operator()(const std::vector<TangentVector>& vecs) const {
    _logger->info("Exporting {} vectors tangent to a mesh", vecs.size());
    std::vector<rrc::Position3D> origins;
    std::vector<rrc::Vector3D>   tips;
    origins.reserve(vecs.size());
    tips.reserve(vecs.size());

    std::transform(
            vecs.cbegin(),
            vecs.cend(),
            std::back_inserter(origins),
            [this](const TangentVector& v) {
                return (*this)(v.application_point().position());
            }
    );
    std::transform(
            vecs.cbegin(),
            vecs.cend(),
            std::back_inserter(tips),
            [this](const TangentVector& v) { return (*this)(v.cartesian_vector()); }
    );
    return rra::Arrows3D::from_vectors(std::move(tips))
            .with_origins(std::move(origins));
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//

rrd::Vec3D
RerunConverter::operator()(const Eigen::Vector3d& x) const {
    return {static_cast<float>(x(0)), static_cast<float>(x(1)), static_cast<float>(x(2))
    };
}

std::vector<rrc::Vector3D>
RerunConverter::mesh_vertex_normals(const Mesh& mesh) const {
    std::vector<rrc::Vector3D> normals;
    normals.reserve(mesh.num_vertices());
    std::transform(
            mesh.vertices_begin(),
            mesh.vertices_end(),
            std::back_inserter(normals),
            [this](const Mesh::Vertex& v) -> rrd::Vec3D { return (*this)(v.normal()); }
    );
    return normals;
}

std::vector<rrc::TriangleIndices>
RerunConverter::mesh_triangles(const Mesh& mesh) const {
    std::vector<rrc::TriangleIndices> res;
    res.reserve(mesh.num_faces());
    std::transform(
            mesh.faces_begin(),
            mesh.faces_end(),
            std::back_inserter(res),
            [](const Mesh::Face& f) -> rrc::TriangleIndices {
                const auto [id1, id2, id3] = f.vertices_ids();
                return {static_cast<std::uint32_t>(id1),
                        static_cast<std::uint32_t>(id2),
                        static_cast<std::uint32_t>(id3)};
            }
    );
    return res;
}

std::vector<rrc::Position3D>
RerunConverter::mesh_vertices(const Mesh& mesh) const {
    _logger->debug("Converting vertices");
    std::vector<rrc::Position3D> vertices;
    vertices.reserve(mesh.num_vertices());
    std::transform(
            mesh.vertices_begin(),
            mesh.vertices_end(),
            std::back_inserter(vertices),
            [this](const Mesh::Vertex& v) { return (*this)(v.position()); }
    );
    return vertices;
};
