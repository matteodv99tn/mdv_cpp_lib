#include "mdv/rerun.hpp"

#include <spdlog/spdlog.h>
#include <stdexcept>

#include <rerun/archetypes/mesh3d.hpp>
#include <rerun/archetypes/points3d.hpp>
#include <rerun/archetypes/series_point.hpp>
#include <rerun/collection.hpp>
#include <rerun/components/position3d.hpp>
#include <rerun/components/triangle_indices.hpp>
#include <rerun/components/vector3d.hpp>

#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/utils/logging.hpp"
#include "mdv/utils/logging_extras.hpp"

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

rra::LineStrips3D
RerunConverter::operator()(const mdv::mesh::Geodesic& geod) const {
    _logger->info("Exporting geodesic line");
    if (geod.empty()) {
        _logger->warn("Geodesic line has no points!");
        throw std::runtime_error("Not enough points");
    }

    std::vector<rrc::Position3D> points;

    points.reserve(geod.size());
    for (const auto& pt : geod) points.emplace_back(operator()(pt));

    return rra::LineStrips3D(std::vector({rrc::LineStrip3D(std::move(points))}));
}

rra::Points3D
RerunConverter::operator()(const Mesh::Point& pt) const {
    _logger->debug(
            "Exporting point on a mesh at position {}", eigen_to_str(pt.position())
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
    for (const auto& v : vecs) {
        origins.emplace_back(operator()(v.application_point().position()));
        tips.emplace_back(operator()(v.cartesian_vector()));
    }

    return rra::Arrows3D::from_vectors(std::move(tips))
            .with_origins(std::move(origins));
}

rra::Arrows3D
RerunConverter::operator()(
        const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori, const double scale
) const {
    rrd::Vec3D dx = operator()(ori* Eigen::Vector3d::UnitX() * scale);
    rrd::Vec3D dy = operator()(ori* Eigen::Vector3d::UnitY() * scale);
    rrd::Vec3D dz = operator()(ori* Eigen::Vector3d::UnitZ() * scale);
    rrd::Vec3D o  = operator()(pos);

    return rra::Arrows3D::from_vectors({dx, dy, dz})
            .with_origins({o, o, o})
            .with_colors(
                    {rrc::Color(255, 0, 0, 255),
                     rrc::Color(0, 255, 0, 255),
                     rrc::Color(0, 0, 255, 255)}
            );
}

rra::Points3D
RerunConverter::as_points(const std::vector<Eigen::Vector3d>& pts) const {
    std::vector<rrc::Position3D> pos;
    pos.reserve(pts.size());
    for (const auto& p : pts) pos.emplace_back(operator()(p));
    return rra::Points3D(std::move(pos));
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//

rrd::Vec3D
RerunConverter::operator()(const Eigen::Vector3d& x) const {
    return {static_cast<float>(x(0)),
            static_cast<float>(x(1)),
            static_cast<float>(x(2))};
}

std::vector<rrc::Vector3D>
RerunConverter::mesh_vertex_normals(const Mesh& mesh) const {
    std::vector<rrc::Vector3D> res;

    res.reserve(mesh.num_vertices());
    for (const auto& v : mesh.vertices()) res.emplace_back(operator()(v.normal()));

    return res;
}

std::vector<rrc::TriangleIndices>
RerunConverter::mesh_triangles(const Mesh& mesh) const {
    std::vector<rrc::TriangleIndices> res;

    res.reserve(mesh.num_faces());
    for (const auto& f : mesh.faces()) {
        const auto [id1, id2, id3] = f.vertices_ids();
        res.emplace_back(id1, id2, id3);
    }
    return res;
}

std::vector<rrc::Position3D>
RerunConverter::mesh_vertices(const Mesh& mesh) const {
    _logger->debug("Converting vertices");
    std::vector<rrc::Position3D> res;

    res.reserve(mesh.num_vertices());
    for (const auto& v : mesh.vertices()) res.emplace_back(operator()(v.position()));

    return res;
};
