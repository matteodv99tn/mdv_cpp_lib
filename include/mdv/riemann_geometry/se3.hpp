#ifndef MDV_SE3_MANIFOLD_HPP
#define MDV_SE3_MANIFOLD_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Transform.h>
#include <utility>

#include "mdv/riemann_geometry/fwd.hpp"
#include "mdv/riemann_geometry/utils.hpp"

namespace mdv::riemann {

struct SE3Point {
    Eigen::Vector3d    pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond ori = Eigen::Quaterniond::Identity();

    SE3Point() = default;

    SE3Point(const Eigen::Vector3d& pos_, const Eigen::Quaterniond& ori_) :
            pos(pos_), ori(ori_) {
        // if (ori.w() < 0.0) ori.coeffs() *= -1;
    }

    Eigen::Affine3d to_affine() const;

    static SE3Point from_affine(const Eigen::Affine3d& transform);
};

struct SE3TangentVector {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector4d ori = Eigen::Vector4d::Zero();

    SE3TangentVector() = default;

    SE3TangentVector(Eigen::Vector3d linear_vel, Eigen::Vector4d angular_vel) :
            pos(std::move(linear_vel)), ori(std::move(angular_vel)) {}

    SE3TangentVector(const Eigen::Vector<double, 7>& vec) {
        pos = vec.head(3);
        ori = vec.tail(4);
    };
};

inline SE3TangentVector
operator+(const SE3TangentVector& v1, const SE3TangentVector& v2) {
    SE3TangentVector res;
    res.pos = v1.pos + v2.pos;
    res.ori = v1.ori + v2.ori;
    return res;
}

inline SE3TangentVector
operator-(const SE3TangentVector& v1, const SE3TangentVector& v2) {
    SE3TangentVector res;
    res.pos = v1.pos - v2.pos;
    res.ori = v1.ori - v2.ori;
    return res;
}

inline SE3TangentVector
operator*(const SE3TangentVector& vec, const double s) {
    SE3TangentVector res;
    res.pos = vec.pos * s;
    res.ori = vec.ori * s;
    return res;
}

inline SE3TangentVector
operator*(const double s, const SE3TangentVector& vec) {
    SE3TangentVector res;
    res.pos = vec.pos * s;
    res.ori = vec.ori * s;
    return res;
}

inline SE3TangentVector
operator/(const SE3TangentVector& vec, const double s) {
    SE3TangentVector res;
    res.pos = vec.pos / s;
    res.ori = vec.ori / s;
    return res;
}

inline bool
operator==(const SE3TangentVector& v1, const SE3TangentVector& v2) {
    return v1.pos == v2.pos && v1.ori == v2.ori;
}

class SE3 {
public:
    using Point         = SE3Point;
    using TangentVector = SE3TangentVector;

    static TangentVector logarithmic_map(const Point& q1, const Point& q2);

    static Point exponential_map(const Point& q, const TangentVector& v);

    static TangentVector parallel_transport(
            const Point& q, const Point& p, const TangentVector& v
    );

    static TangentVector covariant_derivative(const Point& q, const TangentVector& v);

    static Point default_point();

    static TangentVector default_tangent_vector();
};

template <>
struct TrivialTypeEmbedding<riemann::SE3TangentVector> {
    using Input  = riemann::SE3TangentVector;
    using Output = Eigen::Vector<double, 7>;

    Output
    embed(const Input& in) const {
        return {in.pos(0),
                in.pos(1),
                in.pos(2),
                in.ori(0),
                in.ori(1),
                in.ori(2),
                in.ori(3)};
    }

    Input
    decode(const Output& out) const {
        return out;
    }
};

}  // namespace mdv::riemann

#endif  // MDV_SE3_MANIFOLD_HPP
