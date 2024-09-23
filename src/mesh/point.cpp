#include <mdv/mesh/face.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/point.hpp>
#include <mdv/mesh/vertex.hpp>

#include "cgal_data.hpp"

using mdv::mesh::Face;
using mdv::mesh::Point;
using mdv::mesh::Vertex;

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//

Point::Point(const Face& face, const Vec2_t& uv_coords) :
        _face(face), _uv_coords(uv_coords) {
}

Point
Point::from_cartesian_position(const Point3d_t& point, const MeshElement& mesh) {
    const CgalData::Point3_t cgal_point{point.x(), point.y(), point.z()};
    const auto [face_id, bar_coords] = mesh.cmesh()->_cgal_data->_shortest_path->locate(
            cgal_point, mesh.cmesh()->_cgal_data->_aabb_tree
    );

    const Face face{mesh, static_cast<FaceIndex_t>(face_id)};

    const auto& [v1, v2, v3] = face.vertices();
    Eigen::Matrix<double, 3, 2> A;  // NOLINT
    A.col(0)       = v2.position() - v1.position();
    A.col(1)       = v3.position() - v1.position();
    const Vec3_t b = point - v1.position();
    const Vec2_t x = A.colPivHouseholderQr().solve(b);
    return {face, x};
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __ ___
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
// | |  | |  __/ | | | | | |_) |  __/ |  \__ \
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
//

mdv::mesh::Point3d_t
Point::position() const {
    const auto [vert0, vert1, vert2] = _face.vertices();
    const auto p0                    = vert0.position();
    const auto p1                    = vert1.position();
    const auto p2                    = vert2.position();
    const auto e0                    = p1 - p0;
    const auto e1                    = p2 - p0;
    return p0 + _uv_coords(0) * e0 + _uv_coords(1) * e1;
};
