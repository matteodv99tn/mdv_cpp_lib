#ifdef MDV_WITH_RERUN_SDK
#ifndef MDV_RERUN_INTEGRATION_HPP
#define MDV_RERUN_INTEGRATION_HPP

#include <rerun/archetypes/arrows3d.hpp>
#include <rerun/archetypes/line_strips3d.hpp>
#include <rerun/archetypes/mesh3d.hpp>
#include <rerun/archetypes/points3d.hpp>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/utils/logging.hpp"

namespace mdv {


class RerunConverter {
public:
    MDV_NODISCARD
    ::rerun::archetypes::Mesh3D operator()(const ::mdv::mesh::Mesh&) const;

    MDV_NODISCARD ::rerun::archetypes::LineStrips3D
    operator()(const ::mdv::mesh::Geodesic&) const;

    MDV_NODISCARD ::rerun::archetypes::Points3D
    operator()(const ::mdv::mesh::Mesh::Point&) const;

    MDV_NODISCARD ::rerun::archetypes::Points3D
    operator()(const std::vector<::mdv::mesh::Mesh::Point>&) const;

    MDV_NODISCARD ::rerun::archetypes::Arrows3D
    operator()(const std::vector<::mdv::mesh::TangentVector>&) const;

protected:
    MDV_NODISCARD
    std::vector<::rerun::components::Vector3D>
    mesh_vertex_normals(const ::mdv::mesh::Mesh&) const;

    MDV_NODISCARD
    std::vector<::rerun::components::Position3D> mesh_vertices(const ::mdv::mesh::Mesh&)
            const;

    MDV_NODISCARD
    std::vector<::rerun::components::TriangleIndices>
    mesh_triangles(const ::mdv::mesh::Mesh&) const;


    MDV_NODISCARD
    ::rerun::datatypes::Vec3D operator()(const Eigen::Vector3d&) const;

    static SpdLoggerPtr _logger;
};

class MeshRerunConverter : public RerunConverter {};

}  // namespace mdv

#endif  // MDV_RERUN_INTEGRATION_HPP
#endif  // MDV_WITH_RERUN_SDK
