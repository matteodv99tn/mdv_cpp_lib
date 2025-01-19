#ifndef MDV_RIEMANN_MANIFOLD_HPP
#define MDV_RIEMANN_MANIFOLD_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>

#define MDV_MANIFOLD_TYPENAMES_IMPORT(M)                                               \
    using Point         = typename M::Point;                                           \
    using TangentVector = typename M::TangentVector;

namespace mdv::riemann {

template <typename M>
concept manifold = requires(M m, M::Point x, M::TangentVector v) {
    typename M::Point;
    typename M::TangentVector;

    { M::logarithmic_map(x, x) } -> std::same_as<typename M::TangentVector>;
    { M::exponential_map(x, v) } -> std::same_as<typename M::Point>;
    { M::parallel_transport(x, x, v) } -> std::same_as<typename M::TangentVector>;
};

class S3 {
public:
    using Point         = Eigen::Quaterniond;
    using TangentVector = Eigen::Vector4d;

    static Eigen::Vector4d
    logarithmic_map(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
        return q2.coeffs() - q1.coeffs();
    }

    static Point
    exponential_map(const Point& q, const TangentVector& v) {
        return Eigen::Quaterniond(q.coeffs() + v);
    }

    static TangentVector
    parallel_transport(const Point, const Point, const TangentVector) {
        throw std::domain_error("unimplemented S3::parallel_transport");
    }
};

class Scalar {
public:
    using Point         = double;
    using TangentVector = double;

    static double
    logarithmic_map(const double& p1, const double& p2) {
        return p2 - p1;
    }

    static double
    exponential_map(const double& p, const double& v) {
        return p + v;
    }

    static TangentVector
    parallel_transport(const double p1, const double p2, const double v1) {
        return v1;
    }
};


}  // namespace mdv::riemann


#endif  // MDV_RIEMANN_MANIFOLD_HPP
