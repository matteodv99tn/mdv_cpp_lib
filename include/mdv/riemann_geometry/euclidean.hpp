#ifndef MDV_EUCLIDEAN_MANIFOLD_HPP
#define MDV_EUCLIDEAN_MANIFOLD_HPP

#include <Eigen/Dense>

namespace mdv::riemann {

template <int Dim>
class Rn {
public:
    using Point         = Eigen::Vector<double, Dim>;
    using TangentVector = Eigen::Vector<double, Dim>;

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

    static double
    covariant_derivative(const double& /* unused */, const double& v) {
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
