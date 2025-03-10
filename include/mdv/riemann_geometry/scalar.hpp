#ifndef MDV_SCALAR_MANIFOLD_HPP
#define MDV_SCALAR_MANIFOLD_HPP

namespace mdv::riemann {

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

    static double
    covariant_derivative(const double& /* unused */, const double& v) {
        return v;
    }

    static Point
    default_point() {
        return 0;
    }

    static TangentVector
    default_tangent_vector() {
        return 0;
    }
};


}  // namespace mdv::riemann


#endif  // MDV_SCALAR_MANIFOLD_HPP
