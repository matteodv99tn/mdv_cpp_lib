#ifndef MDV_RIEMANN_MANIFOLD_HPP
#define MDV_RIEMANN_MANIFOLD_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>

namespace mdv::riemann {

template <typename M>
concept manifold = requires(M m, M::Type x, M::Tangent v) {
    typename M::Type;
    typename M::Tangent;

    { M::logarithmic_map(x, x) } -> std::same_as<typename M::Tangent>;
    { M::exponential_map(x, v) } -> std::same_as<typename M::Type>;
    { M::parallel_transport(x, x, v) } -> std::same_as<typename M::Tangent>;
};

class S3 {
public:
    using Type    = Eigen::Quaterniond;
    using Tangent = Eigen::Vector4d;

    static Eigen::Vector4d
    logarithmic_map(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
        return q2.coeffs() - q1.coeffs();
    }

    static Type
    exponential_map(const Type& q, const Tangent& v) {
        return Eigen::Quaterniond(q.coeffs() + v);
    }

    static Tangent
    parallel_transport(const Type, const Type, const Tangent) {
        throw std::domain_error("unimplemented S3::parallel_transport");
    }
};


}  // namespace mdv::riemann


#endif  // MDV_RIEMANN_MANIFOLD_HPP
