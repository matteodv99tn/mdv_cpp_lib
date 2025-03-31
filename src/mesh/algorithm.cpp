#include "mdv/mesh/algorithm.hpp"

#include <spdlog/spdlog.h>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/conditions.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

using mdv::mesh::Geodesic;
using mdv::mesh::Mesh;
using mdv::mesh::TangentVector;

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

TangentVector
mdv::mesh::parallel_transport(const TangentVector& v, const Mesh::Point& p) {
    auto trihedron = [](const Mesh::Point& pt, const Vec3d& dir) -> Eigen::Matrix3d {
        Eigen::Matrix3d res;
        res.col(0) = dir;
        res.col(2) = pt.face().normal();
        res.col(1) = res.col(2).cross(res.col(0));
        return res;
    };
    p.logger().debug(
            "Computing parallel transport of vector {} applied in {} to target point "
            "{}",
            eigen_to_str(v.application_point().position()),
            eigen_to_str(v.cartesian_vector()),
            eigen_to_str(p.position())
    );

    if (v.application_point().face() == p.face()) return {p, v.uv()};

    const Mesh::Point& o = v.application_point();
    assert(o.data().impl);
    const Geodesic geod =
            internal::construct_geodesic(*o.data().impl, v.application_point(), p);


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
    require_on_same_mesh(p.face(), y.face());

    p.logger().debug(
            "Computing logarithmic map of point {} w.r.t. point {}",
            eigen_to_str(y.position()),
            eigen_to_str(p.position())
    );

    if (p.face().id() == y.face().id())
        return {p, Mesh::Point::UvCoord(y.uv() - p.uv())};

    const auto geod        = internal::construct_geodesic(p.cgal(), p, y);
    auto       log_map_dir = (geod[1] - geod[0]).normalized();
    auto       log_map_len = length(geod);
    return {p, Vec3d(log_map_len * log_map_dir)};
}

Mesh::Point
mdv::mesh::exponential_map(TangentVector v, Geodesic* geod) {
    v.logger().debug(
            "Computing the exponential map from point {} with tangent vector {}",
            eigen_to_str(v.application_point().position()),
            eigen_to_str(v.cartesian_vector())
    );

    if (geod) geod->emplace_back(v.application_point().position());

    if (condition::is_zero_norm(v.uv())) return v.application_point();

    std::size_t count = 0;
    while (!condition::is_zero_norm(v.uv()) || (count < 1000)) {
        if (geod) geod->emplace_back(v.application_point().position());
        const auto trimmed_vec = v.trim();

        // Check if trimming did not went to another face
        if (trimmed_vec == std::nullopt) {
            if (geod) geod->emplace_back(v.tip());
            const TangentVector::UvCoord target_uv =
                    v.application_point().uv() + v.uv();
            return Mesh::Point(v.application_point().face(), target_uv);
        }

        v = trimmed_vec.value();
        ++count;
    }

    throw std::runtime_error("Exceeded iteration limit");
}

double
mdv::mesh::distance(const Mesh::Face& f, const CartesianPoint& pt) {
    const auto delta = pt - f.vertex(0).position();
    return std::abs(delta.dot(f.normal()));
}

double
mdv::mesh::distance(const Mesh::Point& p1, const Mesh::Point& p2) {
    return (p1.position() - p2.position()).norm();
}

std::pair<Mesh::Vertex, Mesh::Vertex>
mdv::mesh::shared_vertices(const Mesh::Face& f1, const Mesh::Face& f2) {
    require_on_same_mesh(f1, f2);

    std::vector<Mesh::Vertex::Index> indices;
    indices.reserve(3);

    const auto f2_vbeg = f2.vertices_ids().begin();
    const auto f2_vend = f2.vertices_ids().end();

    const auto v1_check = std::find(f2_vbeg, f2_vend, f1.vertex(0).id());
    if (v1_check != f2_vend) indices.emplace_back(*v1_check);

    const auto v2_check = std::find(f2_vbeg, f2_vend, f1.vertex(1).id());
    if (v2_check != f2_vend) indices.emplace_back(*v2_check);

    const auto v3_check = std::find(f2_vbeg, f2_vend, f1.vertex(2).id());
    if (v3_check != f2_vend) indices.emplace_back(*v3_check);

    if (indices.size() != 2) {
        throw std::runtime_error(
                "yield_shared_vertices: expecting 2 matches, found "
                + std::to_string(indices.size())
        );
    }
    return {Mesh::Vertex(f1.data(), indices[0]), Mesh::Vertex(f1.data(), indices[1])};
}

bool
mdv::mesh::uv_in_unitary_triangle(const Eigen::Vector2d& uv) {
    return (uv.sum() <= 1.0) && (uv(0) >= 0.0) && (uv(1) >= 0.0);
}
