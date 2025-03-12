#ifndef MDV_EUCLIDEAN_MANIFOLD_HPP
#define MDV_EUCLIDEAN_MANIFOLD_HPP

#include <Eigen/Dense>

namespace mdv::riemann {

template <int Dim>
class Rn {
public:
    using Point         = Eigen::Vector<double, Dim>;
    using TangentVector = Eigen::Vector<double, Dim>;

    static TangentVector
    logarithmic_map(const Point& p1, const Point& p2) {
        return p2 - p1;
    }

    static Point
    exponential_map(const Point& p, const TangentVector& v) {
        return p + v;
    }

    static TangentVector
    parallel_transport(const Point& p1, const Point& p2, const TangentVector& v1) {
        return v1;
    }

    static TangentVector
    covariant_derivative(const Point& /* unused */, const TangentVector& v) {
        return v;
    }

    static Point
    default_point() {
        return Point::Zero();
    }

    static TangentVector
    default_tangent_vector() {
        return Point::Zero();
    }
};


}  // namespace mdv::riemann


#endif  // MDV_EUCLIDEAN_MANIFOLD_HPP
