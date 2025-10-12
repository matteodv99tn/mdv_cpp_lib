#include "mdv/riemann_geometry/se3.hpp"

#include <Eigen/src/Geometry/Transform.h>

#include "mdv/riemann_geometry/euclidean.hpp"
#include "mdv/riemann_geometry/s3.hpp"

using SE3 = mdv::riemann::SE3;
using R3  = mdv::riemann::Rn<3>;
using S3  = mdv::riemann::S3;

Eigen::Affine3d
mdv::riemann::SE3Point::to_affine() const {
    Eigen::Affine3d res = Eigen::Affine3d::Identity();
    res.translate(pos);
    res.rotate(ori);
    return res;
}

mdv::riemann::SE3Point
mdv::riemann::SE3Point::from_affine(const Eigen::Affine3d& transform) {
    return {transform.translation(), Eigen::Quaterniond(transform.rotation())};
}

SE3::TangentVector
SE3::logarithmic_map(const Point& p1, const Point& p2) {
    return {R3::logarithmic_map(p1.pos, p2.pos), S3::logarithmic_map(p1.ori, p2.ori)};
}

SE3::Point
SE3::exponential_map(const Point& p, const TangentVector& v) {
    return {R3::exponential_map(p.pos, v.pos), S3::exponential_map(p.ori, v.ori)};
}

SE3::TangentVector
SE3::parallel_transport(const Point& q, const Point& p, const TangentVector& v) {
    return {R3::parallel_transport(q.pos, p.pos, v.pos),
            S3::parallel_transport(q.ori, p.ori, v.ori)};
}

SE3::TangentVector
SE3::covariant_derivative(const Point& p, const TangentVector& v) {
    return {R3::covariant_derivative(p.pos, v.pos),
            S3::covariant_derivative(p.ori, v.ori)};
}

SE3::Point
SE3::default_point() {
    return {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
}

SE3::TangentVector
SE3::default_tangent_vector() {
    return {Eigen::Vector3d::Zero(), Eigen::Vector4d::Zero()};
}
