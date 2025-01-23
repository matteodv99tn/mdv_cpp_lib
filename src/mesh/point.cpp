#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <cstdlib>
#include <spdlog/spdlog.h>

#include <mdv/mesh/algorithm.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/utils/conditions.hpp>

#include "cgal_data.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/logging_extras.hpp"

using mdv::mesh::Mesh;

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//

Mesh::Point::Point(const Face& face, const Point3d& pt) :
        _face(face), _uv_map(_face.compute_uv_map()) {
    _uv = uv_map().inverse_map(pt);
}

Mesh::Point
Mesh::Point::from_cartesian(const Mesh& m, const Point3d& pt) {
    const Mesh::CgalData::Point3 cgal_pt{pt(0), pt(1), pt(2)};
    const auto [id, coords] =
            m._data->_shortest_path->locate(cgal_pt, m._data->_aabb_tree);
    const Face face(m, static_cast<Mesh::Face::Index>(id.idx()));
    return {face, pt};
}

Mesh::Point
Mesh::Point::from_face_and_position(const Mesh::Face& f, const Point3d& pt) {
    const double d = distance(f, pt);
    assert(condition::is_zero(d));
    return {f, pt};
}

Mesh::Point
Mesh::Point::undefined(const Mesh& m) noexcept {
    const Face    face{m, -1};
    const UvCoord uv{-1.0, -1.0};
    return {face, uv};
}

Mesh::Point
Mesh::Point::random(const Mesh& m) noexcept {
    const auto face   = Mesh::Face::random(m);
    const auto uv_val = (UvCoord::Ones() + UvCoord::Random());
    // Worst case scenario: uv_val = (2, 2) -> uv = (0.5, 0.5)
    // so, divide by 8
    const UvCoord uv = uv_val / 8;
    Ensures(uv_in_unitary_triangle(uv));
    return {face, uv};
}

bool
Mesh::Point::is_undefined() const noexcept {
    return face_id() == -1 && _uv == UvCoord(-1.0, -1.0);
}

Eigen::Vector3d
Mesh::Point::barycentric() const noexcept {
    const auto& v1 = _face.v1();
    const auto& v2 = _face.v2();
    const auto& v3 = _face.v3();

    Eigen::Matrix3d A;
    A.col(0) = v1;
    A.col(1) = v2;
    A.col(2) = v3;

    const Eigen::Vector3d b   = position();
    const Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
    if (!mdv::condition::is_zero(sol.sum() - 1.0)) {
        logger()->error(
                "When converting from UV coords. to barycentric, sum ({}) is different "
                "1.0",
                sol.sum()
        );
    }
    Ensures(mdv::condition::is_zero(sol.sum() - 1.0));
    return sol;
};

mdv::mesh::Point3d
Mesh::Point::position() const noexcept {
    return uv_map().forward_map(_uv);
}

void
Mesh::Point::constrain_inside_triangle() & {
    if (u() < 0.0) _uv(0) = 0.0;
    if (v() < 0.0) _uv(1) = 0.0;

    // To reduce numerical approximation error, I slightly reduce the length of the uv
    // vector (in this case with a factor 1e-5) which shall be negligible in all cases.
    const double uv_sum = uv().sum();
    if (uv_sum > 1.0) _uv /= uv_sum * (1 + 1e-5);  // NOLINT
    assert(uv_in_unitary_triangle(uv()));
}

Mesh::Point
Mesh::Point::constrain_inside_triangle() && {
    if (u() < 0.0) _uv(0) = 0.0;
    if (v() < 0.0) _uv(1) = 0.0;

    const double uv_sum = uv().sum();
    if (uv_sum > 1.0) _uv /= uv_sum * (1 + 1e-5);  // NOLINT
    assert(uv_in_unitary_triangle(uv()));
    return *this;
}

std::string
Mesh::Point::describe() const {
    return fmt::format("point at face #{}, uv: {}", face().id(), eigen_to_str(uv()));
}

bool
Mesh::Point::operator==(const Point& other) const noexcept {
    const bool same_face = (this->face() == other.face());
    const bool same_uv   = mdv::condition::is_zero_norm(this->uv() - other.uv());
    return same_face && same_uv;
}

bool
Mesh::Point::operator!=(const Point& other) const noexcept {
    return !(*this == other);
}
