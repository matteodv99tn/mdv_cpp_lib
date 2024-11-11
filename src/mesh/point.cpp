#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <cstdlib>

#include <mdv/mesh/mesh.hpp>

#include "cgal_data.hpp"
#include "mdv/mesh/fwd.hpp"

using mdv::mesh::Mesh;

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//

Mesh::Point
Mesh::Point::from_cartesian(const Mesh* m, const Point3d& pt) {
    const Mesh::CgalMesh::Point3 cgal_pt{pt(0), pt(1), pt(2)};
    const auto [id, coords] =
            m->_cgal_data->_shortest_path->locate(cgal_pt, m->_cgal_data->_aabb_tree);

    const Face face(m, static_cast<Mesh::Face::Index>(id.idx()));

    // Convert barycentric coordinates to UV
    const auto&                       v1 = face.v1();
    const auto&                       v2 = face.v2();
    const auto&                       v3 = face.v3();
    const Eigen::Matrix<double, 3, 2> A  = [&v1, &v2, &v3]() {
        Eigen::Matrix<double, 3, 2> mat;
        mat.col(0) = v2 - v1;
        mat.col(1) = v3 - v1;
        return mat;
    }();
    const Eigen::Vector3d b  = pt - v1;
    const Eigen::Vector2d uv = A.fullPivHouseholderQr().solve(b);

    return {face, uv};
}

Mesh::Point
Mesh::Point::undefined(const Mesh* m) noexcept {
    const Face            face{m, -1};
    const Eigen::Vector2d uv{-1.0, -1.0};
    return {face, uv};
}

Mesh::Point
Mesh::Point::random(const Mesh* m) noexcept {
    const Face face{m, static_cast<Mesh::Face::Index>(std::rand() % m->num_faces())};
    const auto uv =
            (Eigen::Vector2d::Ones() + Eigen::Vector2d::Random()) / (2 * std::sqrt(2));
    Ensures(uv.sum() <= 1.0);
    Ensures(uv(0) >= 0.0);
    Ensures(uv(1) >= 0.0);
    return {face, uv};
}

bool
Mesh::Point::is_undefined() const noexcept {
    return face_id() == -1 && _uv == Eigen::Vector2d(-1.0, -1.0);
}

Eigen::Vector3d
Mesh::Point::barycentric() const noexcept {
    const auto& v1 = _face.v1();
    const auto& v2 = _face.v2();
    const auto& v3 = _face.v3();

    const Eigen::Matrix3d A = [&v1, &v2, &v3]() {
        Eigen::Matrix3d mat;
        mat.row(0) = v1;
        mat.row(1) = v2;
        mat.row(2) = v3;
        return mat;
    }();
    const Eigen::Vector3d b = position();
    return A.colPivHouseholderQr().solve(b);
};

mdv::mesh::Point3d
Mesh::Point::position() const noexcept {
    const auto& p1 = _face.v1();
    const auto& p2 = _face.v2();
    const auto& p3 = _face.v3();
    return p1 + (p2 - p1) * _uv(0) + (p3 - p1) * _uv(1);
}
