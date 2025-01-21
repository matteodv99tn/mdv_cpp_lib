#include <mdv/eigen_defines.hpp>
#include <mdv/mesh/algorithm.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/tangent_vector.hpp>
#include <mdv/utils/conditions.hpp>

using mdv::mesh::Mesh;
using mdv::mesh::TangentVector;

double
mdv::mesh::length(const Geodesic& geod) {
    double res = 0;
    for (auto it = geod.cbegin(); it != geod.cend() - 1; ++it)
        res += (*it - *(it + 1)).norm();
    return res;
}

TangentVector
mdv::mesh::parallel_transport(const TangentVector& v, const Mesh::Point& p) {
    auto trihedron = [](const Mesh::Point& pt, const Vec3d& dir) -> Eigen::Matrix3d {
        Eigen::Matrix3d res;
        res.col(0) = dir;
        res.col(2) = pt.face().normal();
        res.col(1) = res.col(2).cross(res.col(0));
        return res;
    };

    if (v.application_point().face_id() == p.face_id()) return {p, v.uv()};

    const Mesh::Point& o    = v.application_point();
    const auto&        mesh = o.mesh();
    const Geodesic     geod = mesh.build_geodesic(v.application_point(), p);

    const auto  n  = geod.size();
    const auto& x1 = (geod[1] - geod[0]).normalized();
    const auto& x2 = (geod[n - 1] - geod[n - 2]).normalized();
    const auto  R1 = trihedron(o, x1);  // NOLINT
    const auto  R2 = trihedron(p, x2);  // NOLINT
    assert(mdv::condition::is_zero(R1.determinant() - 1.0));
    assert(mdv::condition::is_zero(R2.determinant() - 1.0));

    const Vec3d delta = R2 * R1.transpose() * v.cartesian_vector();
    return TangentVector::from_tip_position(p, p.position() + delta);
}

TangentVector
mdv::mesh::logarithmic_map(const Mesh::Point& p, const Mesh::Point& y) {
    assert(&p.mesh() == &y.mesh());

    if (p.face_id() == y.face_id()) return {p, Mesh::Point::UvCoord(y.uv() - p.uv())};

    const auto& mesh        = p.mesh();
    const auto  geod        = mesh.build_geodesic(p, y);
    auto        log_map_dir = (geod[1] - geod[0]).normalized();
    auto        log_map_len = length(geod);
    return {p, Vec3d(log_map_len * log_map_dir)};
}
