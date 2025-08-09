#include "mdv/mesh/algorithm.hpp"

#include <range/v3/algorithm/for_each.hpp>
#include <range/v3/all.hpp>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/conditions.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

// \cond DOXYGEN_IGNORE
using mdv::mesh::Geodesic;
using mdv::mesh::Mesh;
using mdv::mesh::TangentVector;
// \endcond

namespace rs = ::ranges;
namespace rv = ::ranges::views;

double
mdv::mesh::length(const Geodesic& geod) {
    double res = 0;
    for (auto it = geod.cbegin(); it != geod.cend() - 1; ++it)
        res += (*it - *(it + 1)).norm();
    return res;
}

mdv::mesh::CartesianPoint
mdv::mesh::point_from_geodesic(
        const Geodesic& geod, const double s_input, const double* len
) {
    double       internal_len = 0.0;
    const double s_target     = std::clamp(s_input, 0.0, 1.0);

    if (len == nullptr) {
        internal_len = length(geod);
        len          = &internal_len;
    }

    double s_travelled = 0.0;

    for (auto i = 0; i < geod.size() - 1; ++i) {
        const Eigen::Vector3d delta = geod[i + 1] - geod[i];
        const double          si    = delta.norm() / (*len);
        if (s_travelled + si < s_target) {
            s_travelled += si;
        } else {
            const double s_left = s_target - s_travelled;
            return geod[i] + delta * s_left / si;
        }
    }

    return geod.back();
}

Geodesic
mdv::mesh::geodesic_resample(const Geodesic& geod, std::vector<double> coordinates) {
    Geodesic     res;
    const double len = length(geod);
    res.reserve(coordinates.size());
    rs::transform(coordinates, std::back_inserter(res), [geod, len](const double s) {
        return point_from_geodesic(geod, s, &len);
    });
    return res;
};

TangentVector
mdv::mesh::parallel_transport(const TangentVector& v, const Point& p) {
    auto trihedron = [](const Point& pt, const Vec3d& dir) -> Eigen::Matrix3d {
        Eigen::Matrix3d res;
        res.col(0) = dir;
        res.col(2) = pt.face().normal();
        res.col(1) = res.col(2).cross(res.col(0));
        return res;
    };

    assert(Mesh::default_logger);
    mdv::Logger& logger = *Mesh::default_logger.get();

    logger.debug(
            "Computing parallel transport of vector {} applied in {} to target "
            "point "
            "{}",
            eigen_to_str(v.application_point().position()),
            eigen_to_str(v.cartesian_vector()),
            eigen_to_str(p.position())
    );

    if (v.application_point().face() == p.face()) return {p, v.uv()};

    const Point& o = v.application_point();

    const Geodesic geod = internal::construct_geodesic(
            o.face().mesh().cgal(), v.application_point(), p
    );


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
mdv::mesh::logarithmic_map(const Point& p, const Point& y) {
    assert(Mesh::default_logger);
    mdv::Logger& logger = *Mesh::default_logger.get();
    logger.debug(
            "Computing logarithmic map of point {} w.r.t. point {}",
            eigen_to_str(y.position()),
            eigen_to_str(p.position())
    );

    if (p.face() == y.face()) return {p, Point::UvCoord(y.uv() - p.uv())};

    const auto geod        = internal::construct_geodesic(p.face().mesh().cgal(), p, y);
    auto       log_map_dir = (geod[1] - geod[0]).normalized();
    auto       log_map_len = length(geod);
    return {p, Vec3d(log_map_len * log_map_dir)};
}

mdv::mesh::Point
mdv::mesh::exponential_map(TangentVector v, Geodesic* geod) {
    assert(Mesh::default_logger);
    mdv::Logger& logger = *Mesh::default_logger.get();
    logger.debug(
            "Computing the exponential map from point {} with tangent vector {}",
            eigen_to_str(v.application_point().position()),
            eigen_to_str(v.cartesian_vector())
    );

    if (geod) geod->emplace_back(v.application_point().position());

    if (condition::is_zero_norm(v.uv())) return v.application_point();

    std::size_t count = 0;
    while (!condition::is_zero_norm(v.uv()) && (count < 1000)) {
        if (geod) geod->emplace_back(v.application_point().position());
        const auto trimmed_vec = v.trim();

        // Check if trimming did not went to another face
        if (trimmed_vec == std::nullopt) {
            if (geod) geod->emplace_back(v.tip());
            const TangentVector::UvCoord target_uv =
                    v.application_point().uv() + v.uv();
            return Point(v.application_point().face(), target_uv);
        }

        v = trimmed_vec.value();
        ++count;
    }

    throw std::runtime_error("Exceeded iteration limit");
}

double
mdv::mesh::distance(const Face& f, const CartesianPoint& pt) {
    const Eigen::Vector3d delta = pt - f.half_edge()->origin_position();
    return std::abs(delta.dot(f.normal()));
}

double
mdv::mesh::distance(const Point& p1, const Point& p2) {
    return (p1.position() - p2.position()).norm();
}

bool
mdv::mesh::uv_in_unitary_triangle(const Eigen::Vector2d& uv) {
    return (uv.sum() <= 1.0) && (uv(0) >= 0.0) && (uv(1) >= 0.0);
}
