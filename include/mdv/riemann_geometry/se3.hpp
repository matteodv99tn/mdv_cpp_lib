#ifndef MDV_S3_MANIFOLD_HPP
#define MDV_S3_MANIFOLD_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mdv::riemann {

struct SE3Point {
    Eigen::Vector3d    pos;
    Eigen::Quaterniond ori;
};

struct SE3TangentVector {
    Eigen::Vector3d pos;
    Eigen::Vector4d ori;
};

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
}  // namespace mdv::riemann

#endif  // MDV_S3_MANIFOLD_HPP
