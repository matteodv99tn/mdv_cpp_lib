#include "mdv/mesh/point.hpp"

#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <cstdlib>
#include <Eigen/Core>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

// \cond DOXYGEN_IGNORE
using mdv::mesh::CartesianPoint;
using mdv::mesh::Face;
using mdv::mesh::Mesh;
using mdv::mesh::Point;
using mdv::mesh::internal::CgalImpl;

// \endcond

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//

Point::Point(const Face& face, const UvCoord& uv) : _face(&face), _uv(uv) {
    assert(mdv::condition::is_zero(mdv::mesh::distance(face, position())));
}

Point::Point(const Face& face, const CartesianPoint& pt) : _face(&face) {
    _uv = uv_map().inverse_map(pt);
    assert(mdv::condition::is_zero(mdv::mesh::distance(face, position())));
}

Point
Point::from_cartesian(const Mesh& m, const CartesianPoint& pt) {
    const CgalImpl::Point3 cgal_pt{pt(0), pt(1), pt(2)};
    const auto [id, coords] =
            m.cgal()._shortest_path->locate(cgal_pt, m.cgal()._aabb_tree);

    const Face& face = m.face(static_cast<Index>(id.idx()));
    return {face, pt};
}

Point
Point::from_face_and_position(const Face& f, const CartesianPoint& pt) {
    // const double d = distance(f, pt);
    // assert(condition::is_zero(d));
    return {f, pt};
}

Point
Point::undefined(const Mesh& m) noexcept {
    const Face&   face = Face::invalid_face;
    const UvCoord uv{-1.0, -1.0};
    return {face, uv};
}

Point
Point::random(const Mesh& m) noexcept {
    const auto& face   = m.random_face();
    const auto  uv_val = (UvCoord::Ones() + UvCoord::Random());
    // Worst case scenario: uv_val = (2, 2) -> uv = (0.5, 0.5)
    // so, divide by 8
    const UvCoord uv = uv_val / 8;
    Ensures(uv_in_unitary_triangle(uv));


    const auto pt = face.uv_map().forward_map(uv_val);
    assert(mdv::condition::is_zero(mdv::mesh::distance(face, pt)));
    return {face, uv};
}

bool
Point::is_undefined() const noexcept {
    return _face->undefined_mesh() && _uv == UvCoord(-1.0, -1.0);
}

Eigen::Vector3d
Point::barycentric() const noexcept {
    using Vec3     = Eigen::Vector3d;
    const auto& he = face().half_edge();
    const auto& a0 = he->origin_position();
    const auto& a1 = he->next()->origin_position();
    const auto& a2 = he->next()->next()->origin_position();
    const Vec3  pt = position();

    const Vec3 v0 = a1 - a0;
    const Vec3 v1 = a2 - a0;
    const Vec3 v2 = pt - a0;

    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);

    const double denom = d00 * d11 - d01 * d01;

    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    const double u = 1.0 - v - w;

    const Vec3 res{u, v, w};


    Eigen::Matrix3d A;
    A.col(0) = a0;
    A.col(1) = a1;
    A.col(2) = a2;

    assert(mdv::condition::are_equal(A * res, pt));
    assert(mdv::condition::is_zero(res.sum() - 1.0));
    return res;
};

mdv::mesh::CartesianPoint
Point::position() const noexcept {
    return uv_map().forward_map(_uv);
}

void
Point::constrain_inside_triangle() & {
    if (u() < 0.0) _uv(0) = 0.0;
    if (v() < 0.0) _uv(1) = 0.0;

    // To reduce numerical approximation error, I slightly reduce the length of the uv
    // vector (in this case with a factor 1e-5) which shall be negligible in all cases.
    const double uv_sum = uv().sum();
    if (uv_sum > 1.0) _uv /= uv_sum * (1 + 1e-5);  // NOLINT
    assert(uv_in_unitary_triangle(uv()));
}

Point
Point::constrain_inside_triangle() && {
    constexpr double zero = 1e-9;
    if (u() < zero) _uv(0) = zero;
    if (v() < zero) _uv(1) = zero;

    const double uv_sum = uv().sum();
    if (uv_sum > 1.0 - zero) _uv /= uv_sum * (1 + 1e-5);  // NOLINT
    assert(uv_in_unitary_triangle(uv()));
    return *this;
}

std::string
Point::describe() const {
    assert(_face);
    return fmt::format("point at {} (f #{})", eigen_to_str(position()), face().id());
}

bool
Point::operator==(const Point& other) const noexcept {
    const bool same_face = (this->face() == other.face());
    const bool same_uv   = mdv::condition::is_zero_norm(this->uv() - other.uv());
    return same_face && same_uv;
}

bool
Point::operator!=(const Point& other) const noexcept {
    return !(*this == other);
}
