#include <spdlog/spdlog.h>

#include <mdv/mesh/tangent_vector.hpp>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

using mdv::mesh::Mesh;
using mdv::mesh::Point3d;
using mdv::mesh::TangentVector;

//  _____                            _ __     __        _
// |_   _|_ _ _ __   __ _  ___ _ __ | |\ \   / /__  ___| |_ ___  _ __
//   | |/ _` | '_ \ / _` |/ _ \ '_ \| __\ \ / / _ \/ __| __/ _ \| '__|
//   | | (_| | | | | (_| |  __/ | | | |_ \ V /  __/ (__| || (_) | |
//   |_|\__,_|_| |_|\__, |\___|_| |_|\__| \_/ \___|\___|\__\___/|_|
//                  |___/
TangentVector::TangentVector(const Point& app_point, const Vec3d& v) :
        _application_point(app_point) {
    assert(mdv::condition::are_orthogonal(app_point.face().normal(), v));
    _uv = jac().householderQr().solve(v);
}

TangentVector
TangentVector::from_tip_position(const Mesh::Point& origin, const Point3d& tip) {
    const auto p0  = origin.position();
    const auto vec = tip - p0;
    const auto n   = origin.face().normal();
    const auto J   = origin.uv_map().forward_map_jacobian();  // NOLINT

    Expects(mdv::condition::are_orthogonal(vec, n));
    return {origin, Eigen::Vector3d(tip - p0)};
}

TangentVector
TangentVector::unit_random(const Mesh::Point& application_point) {
    UvCoord    uv = UvCoord::Random();
    const auto J  = application_point.uv_map().forward_map_jacobian();  // NOLINT
    uv /= (J * uv).norm();
    Ensures(mdv::condition::is_unit_norm(J * uv));
    return {application_point, uv};
}

Point3d
TangentVector::tip() const noexcept {
    return _application_point.position() + cartesian_vector();
}

mdv::Vec3d
TangentVector::cartesian_vector() const noexcept {
    return uv_map().forward_map_jacobian() * _uv;
}

std::optional<TangentVector>
TangentVector::trim(const TrimProjectionFunction& projector) {
    const UvCoord new_uv = _application_point.uv() + uv();

    auto compute_intersection = [p1 = _application_point.uv(), v1 = uv()](
                                        const UvCoord&& p2, const UvCoord&& v2
                                ) -> std::pair<double, double> {
        Eigen::Matrix2d A;  // NOLINT
        A.col(0)     = v1;
        A.col(1)     = -v2;
        const auto b = p2 - p1;

        if (mdv::condition::is_zero(A.determinant())) [[unlikely]]
            return {-1.0, -1.0};
        UvCoord res = A.householderQr().solve(b);
        return {res(0), res(1)};
    };

    const auto& [t1, s1] = compute_intersection({0.0, 0.0}, {1.0, 0.0});
    const auto& [t2, s2] = compute_intersection({0.0, 0.0}, {0.0, 1.0});
    const auto& [t3, s3] = compute_intersection({0.0, 1.0}, {1.0, -1.0});

    using mdv::condition::internal::zero_th;
    int    edge_id = 3;
    double t       = -1.0;

    if ((s1 >= zero_th) && (s1 <= 1.0) && (t1 > zero_th)) {
        edge_id = 1;
        t       = t2;
    } else if ((s2 >= zero_th) && (s2 <= 1.0) && (t2 > zero_th)) {
        assert(edge_id == 3);
        edge_id = 1;
        t       = t2;
    } else if ((s3 >= zero_th) && (s3 <= 1.0) && (t3 > zero_th)) {
        assert(edge_id == 3);
        edge_id = 2;
        t       = t3;
    }
    assert(edge_id != 3);
    assert(t != -1.0);

    if (t >= 1.0)
        return std::nullopt;  // Intersection appears ad a distance greater then the
                              // motion by the vector

    const auto boarder_uv  = _application_point.uv() + t * uv();
    const auto boarder_pos = _application_point.uv_map().forward_map(boarder_uv);
    const auto new_face    = _application_point.face().neighbour_face(edge_id);
    const auto new_app_point =
            Mesh::Point::from_face_and_position(new_face, boarder_pos);

    const auto  uv_delta         = (1.0 - t) * uv();
    const Vec3d cartesian_delta  = uv_map().forward_map_jacobian() * uv_delta;
    const Vec3d projected_vector = projector(new_app_point, cartesian_delta);

    return TangentVector(new_app_point, projected_vector);
}

mdv::Vec3d
TangentVector::default_trim_projection_function(const Point& pt, const Vec3d& vec) {
    const auto& n = pt.face().normal();
    const auto  P = (mdv::Mat3d::Identity() - n * n.transpose());  // projection matrix
    const auto  vec_proj = P * vec;
    return vec_proj.normalized() * vec.norm();
}
