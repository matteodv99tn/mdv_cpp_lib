#ifndef MDV_RERUN_INTEGRATION_HPP
#define MDV_RERUN_INTEGRATION_HPP

#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <rerun/archetypes/arrows3d.hpp>
#include <rerun/archetypes/mesh3d.hpp>
#include <rerun/archetypes/points3d.hpp>
#include <rerun/components/line_strip3d.hpp>

namespace mdv::rerun_convert {

namespace rra = ::rerun::archetypes;
namespace rrc = ::rerun::components;

using ::mdv::mesh::Mesh;

//  __  __           _                           _       _
// |  \/  | ___  ___| |__    _ __ ___   ___   __| |_   _| | ___
// | |\/| |/ _ \/ __| '_ \  | '_ ` _ \ / _ \ / _` | | | | |/ _ \
// | |  | |  __/\__ \ | | | | | | | | | (_) | (_| | |_| | |  __/
// |_|  |_|\___||___/_| |_| |_| |_| |_|\___/ \__,_|\__,_|_|\___|
//
rra::Mesh3D      mesh(const Mesh& mesh);
rrc::LineStrip3D geodesic(const ::mdv::mesh::Geodesic& geodesic);

rra::Arrows3D vertex_normals(const Mesh& mesh);
rra::Points3D point(const Mesh::Point& pt);
rra::Points3D points(const std::vector<Mesh::Point>& pt);

}  // namespace mdv::rerun_convert


#endif  // MDV_RERUN_INTEGRATION_HPP
