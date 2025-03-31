#include "mdv/mesh/tangent_vector.hpp"

#include <spdlog/spdlog.h>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

using mdv::mesh::CartesianPoint;
using mdv::mesh::Mesh;
using mdv::mesh::TangentVector;

//  _____                            _ __     __        _
// |_   _|_ _ _ __   __ _  ___ _ __ | |\ \   / /__  ___| |_ ___  _ __
//   | |/ _` | '_ \ / _` |/ _ \ '_ \| __\ \ / / _ \/ __| __/ _ \| '__|
//   | | (_| | | | | (_| |  __/ | | | |_ \ V /  __/ (__| || (_) | |
//   |_|\__,_|_| |_|\__, |\___|_| |_|\__| \_/ \___|\___|\__\___/|_|
//                  |___/
TangentVector::TangentVector(const Point& app_point, const Vec3d& v) :
        Point(app_point) {
    assert(mdv::condition::are_orthogonal(app_point.face().normal(), v));
    _uv = jac().colPivHouseholderQr().solve(v);
}

TangentVector
TangentVector::from_tip_position(const Mesh::Point& origin, const CartesianPoint& tip) {
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

Eigen::Vector3d
TangentVector::tip() const noexcept {
    return application_point().position() + cartesian_vector();
}

mdv::Vec3d
TangentVector::cartesian_vector() const noexcept {
    return uv_map().forward_map_jacobian() * _uv;
}

std::optional<TangentVector>
TangentVector::trim() {
    using mdv::condition::are_orthogonal;
    logger().trace(
            "Trimming tangent vector with origin '{}', uv: {}",
            application_point().describe(),
            eigen_to_str(uv())
    );

    /**
     * @brief Computes the intersection of two lines given by points and vectors
     *
     * First point (and vector) is always assumed to be current point position and
     * tangent vector direction.
     *
     * Second point and vector are provided as arguments and will represent the
     * different edges of the unitary triangle.
     */
    auto compute_intersection = [p1 = application_point().uv(), v1 = uv()](
                                        const UvCoord&& p2, const UvCoord&& v2
                                ) -> std::pair<double, double> {
        Eigen::Matrix2d A;  // NOLINT
        A.col(0)     = v1;
        A.col(1)     = -v2;
        const auto b = p2 - p1;

        if (mdv::condition::is_zero(A.determinant())) [[unlikely]]
            return {-1.0, -1.0};
        // householderQr shall be faster, but has lower accuracy that is required for
        // the problem
        // UvCoord res = A.householderQr().solve(b);
        UvCoord res = A.colPivHouseholderQr().solve(b);
        return {res(0), res(1)};
    };

    // Compute all possible intersections
    const auto& [t1, s1] = compute_intersection({0.0, 0.0}, {1.0, 0.0});
    const auto& [t2, s2] = compute_intersection({0.0, 0.0}, {0.0, 1.0});
    const auto& [t3, s3] = compute_intersection({0.0, 1.0}, {1.0, -1.0});

    using mdv::condition::internal::zero_th;
    int    edge_id = 3;
    double t       = -1.0;

    // Select valid intersection
    if ((s1 > 0) && (s1 <= 1.0) && (t1 > zero_th)) {
        edge_id = 0;
        t       = t1;
    } else if ((s2 > 0) && (s2 <= 1.0) && (t2 > zero_th)) {
        assert(edge_id == 3);
        edge_id = 1;
        t       = t2;
    } else if ((s3 > 0) && (s3 <= 1.0) && (t3 > zero_th)) {
        assert(edge_id == 3);
        edge_id = 2;
        t       = t3;
    }

    // Ensure uv is not numerically zero, as this could have lead to numerical
    // instability in the computaion of tX and sX
    if (_uv.cwiseAbs().maxCoeff() < 1e-5) return std::nullopt;

    // Validate
    if ((edge_id == 3) || (t == -1.0)) {
        logger().error(
                "TangentVector::trim failed! Could not find a valid intersection to "
                "project remainder of TangentVector"
        );
        logger().warn("Edge id: {}, parameter t = {}", edge_id, t);
        logger().warn(
                "Current application point uv: {} (sum = {})",
                eigen_to_str(application_point().uv()),
                application_point().uv().sum()
        );
        logger().warn("Tangent vector uv: {}", eigen_to_str(uv()));
        logger().warn("t1 = {}, s1 = {}", t1, s1);
        logger().warn("t2 = {}, s2 = {}", t2, s2);
        logger().warn("t3 = {}, s3 = {}", t3, s3);
        throw std::runtime_error("TangentVector::trim failed; check log");
    }

    if (t >= 1.0)
        return std::nullopt;  // Intersection appears ad a distance greater then the
                              // motion by the vector

    // Retrieve new point on the boarder
    const auto boarder_uv  = application_point().uv() + t * uv();
    const auto boarder_pos = application_point().uv_map().forward_map(boarder_uv);
    const auto new_face    = application_point().face().neighbour_face(edge_id);
    const auto new_app_point =
            Mesh::Point::from_face_and_position(new_face, boarder_pos)
                    .constrain_inside_triangle();

    // Compute vector that shall be projected onto the new face
    const auto uv_delta        = (1.0 - t) * uv();
    const auto cartesian_delta = uv_map().forward_map_jacobian() * uv_delta;

    // Compute conformal mapping of cartesian_delta vector
    //  1. find shared edge -> will be the rotation axis
    //  2. find rotation angle based on faces normals (theta)
    //  3. chose between clockwise / counter-clocwise rotation

    const auto [v_shared1, v_shared2] =
            mdv::mesh::shared_vertices(application_point().face(), new_face);
    const auto edge = (v_shared1.position() - v_shared2.position()).normalized();

    const auto   n1        = application_point().face().normal();
    const auto   n2        = new_face.normal();
    const auto   cos_theta = n1.dot(n2);
    const double theta     = std::acos(cos_theta);

    const Eigen::AngleAxis rot1(theta, edge);
    const Eigen::AngleAxis rot2(-theta, edge);
    const auto             res1             = rot1 * cartesian_delta;
    const auto             res2             = rot2 * cartesian_delta;
    const auto             projected_vector = are_orthogonal(res1, n2) ? res1 : res2;
    assert(are_orthogonal(projected_vector, n2));

    return TangentVector(new_app_point, projected_vector);
}

void
TangentVector::scale(const double& factor) {
    _uv *= factor;
}

void
TangentVector::normalise() {
    const double len = (jac() * _uv).norm();
    scale(1.0 / len);
}

TangentVector
TangentVector::normalised() & {
    TangentVector res(*this);
    assert(uv() == res._uv);
    const double len = (jac() * uv()).norm();
    res.scale(1.0 / len);
    return res;
}

TangentVector
TangentVector::normalised() && {
    const double len = (jac() * uv()).norm();
    scale(1.0 / len);
    return *this;
}
