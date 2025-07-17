#include "mdv/mesh/tangent_vector.hpp"

#include <Eigen/src/Core/Matrix.h>

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
TangentVector::from_tip_position(const Point& origin, const CartesianPoint& tip) {
    const auto p0  = origin.position();
    const auto vec = tip - p0;
    const auto n   = origin.face().normal();
    const auto J   = origin.uv_map().forward_map_jacobian();  // NOLINT

    Expects(mdv::condition::are_orthogonal(vec, n));
    return {origin, Eigen::Vector3d(tip - p0)};
}

TangentVector
TangentVector::unit_random(const Point& application_point) {
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
    assert(Mesh::default_logger);
    mdv::Logger& logger = *Mesh::default_logger.get();
    logger.trace(
            "Trimming tangent vector with origin '{}', vector: {}",
            application_point().describe(),
            eigen_to_str(cartesian_vector())
    );

    const auto&  v = cartesian_vector();
    const auto&  p = application_point().position();
    const auto&  f = application_point().face();
    const auto&  n = f.normal();
    const double d = mdv::mesh::distance(f, p);

    assert(mdv::condition::are_orthogonal(v, n));
    assert(mdv::condition::is_zero(d));


    // Ensure uv is not numerically zero, as this could have lead to numerical
    // instability in the computaion of tX and sX
    if (_uv.cwiseAbs().maxCoeff() < 1e-5) return std::nullopt;


    using Vec2  = Eigen::Vector2d;
    using Vec3  = Eigen::Vector3d;
    using Mat32 = Eigen::Matrix<double, 3, 2>;
    Vec3  b;
    Mat32 A;

    HalfEdge*  he = face().half_edge();
    const auto p1 = application_point().position();
    const auto v1 = cartesian_vector();

    bool        first_iter         = true;
    bool        intersection_found = false;
    double      s;
    double      t;
    Vec2        res;
    std::size_t iter = 0;
    while (!intersection_found && (first_iter || he != face().half_edge())) {
        ++iter;
        const auto v2 = he->direction();
        const auto p2 = he->origin_position();
        A.col(0)      = -v1;
        A.col(1)      = v2;
        b             = p1 - p2;
        res           = A.colPivHouseholderQr().solve(b);
        t             = res(0);
        s             = res(1);

        constexpr double zero          = 0.0;
        const bool       intersects_he = (s > -zero) && (s < 1.0 + zero);

        // In the following case the tangent vector is fully contained in the face
        if (intersects_he && t >= 1.0) return std::nullopt;

        intersection_found = (intersects_he && t > -zero);

        if (!intersection_found) he = he->next();
        first_iter = false;
    }

    if (!intersection_found) throw std::runtime_error("Unable to find intersection!");
    logger.trace("Intersection -> s: {}, t: {}", s, t);

    const auto           v2  = he->direction();
    const auto           p2  = he->origin_position();
    const CartesianPoint tmp = p2 + s * v2;

    // Retrieve new point on the boarder
    const CartesianPoint boarder_pos = p1 + t * v1;

    const auto& new_face = he->twin()->face();

    const auto& curr_face = application_point().face();
    const auto  new_app_point =
            Point::from_face_and_position(new_face, tmp).constrain_inside_triangle();


    // Compute vector that shall be projected onto the new face
    const Eigen::Vector3d cartesian_delta = tip() - boarder_pos;

    // Compute conformal mapping of cartesian_delta vector
    const Eigen::Vector3d projected_vector = he->aligning_rotation() * cartesian_delta;
    assert(are_orthogonal(projected_vector, new_face.normal()));

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
