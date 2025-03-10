#ifndef MDV_RIEMANN_HYPERSHPERE_HPP
#define MDV_RIEMANN_HYPERSHPERE_HPP

#include <cmath>
#include <Eigen/Dense>

#include "mdv/utils/conditions.hpp"

namespace mdv::riemann {

template <int Dim>
struct S {
    using Vec = Eigen::Vector<double, Dim + 1>;
    using Mat = Eigen::Matrix<double, Dim + 1, Dim + 1>;

    static Vec
    exponential_map(const Vec& p, const Vec& v) {
        assert(mdv::condition::is_unit_norm(p));
        assert(mdv::condition::are_orthogonal(p, v));
        const double vnorm = v.norm();
        const Vec    res   = p * std::cos(vnorm) + v.normalized() * std::sin(vnorm);
        assert(mdv::condition::is_unit_norm(res));
        return res;
    }

    static Vec
    logarithmic_map(const Vec& p, const Vec& q) {
        assert(mdv::condition::is_unit_norm(p));
        assert(mdv::condition::is_unit_norm(q));
        if (mdv::condition::are_equal(p, q)) return Vec::Zero();
        const Vec    dir      = (q - p.transpose() * q * p).normalized();
        const double distance = std::acos(q.transpose() * p);
        assert(mdv::condition::are_orthogonal(p, dir));
        return dir * distance;
    }

    static Vec
    parallel_transport(const Vec& p, const Vec& q, const Vec& w) {
        assert(mdv::condition::is_unit_norm(p));
        assert(mdv::condition::is_unit_norm(q));
        assert(mdv::condition::are_orthogonal(p, w));

        if (p.isApprox(q)) {
            assert(mdv::condition::are_orthogonal(q, w));
            return w;
        }

        const Vec    v           = logarithmic_map(p, q);
        const double vnorm       = v.norm();
        const Vec    vnormalized = v.normalized();

        const Vec addend1 = (-p * std::sin(vnorm) + vnormalized * std::cos(vnorm))
                            * vnormalized.transpose() * w;
        const Vec addend2 =
                (Mat::Identity() - vnormalized * vnormalized.transpose()) * w;

        // Perform an additional normal projection to reduce numerical stability issues.
        const Vec res = normal_projection(q, addend1 + addend2);

        assert(mdv::condition::are_orthogonal(q, addend1));
        assert(mdv::condition::are_orthogonal(q, addend2));
        assert(mdv::condition::are_orthogonal(q, res));
        return res;
    }

    static Vec
    covariant_derivative(const Vec& q, const Vec& v) {
        return normal_projection(q, v);
    }

    static Vec
    normal_projection(const Vec& n, const Vec& v) {
        assert(mdv::condition::is_unit_norm(n));
        const Vec res = (Mat::Identity() - n * n.transpose()) * v;
        assert(mdv::condition::are_orthogonal(n, res));
        return res;
    }
};


}  // namespace mdv::riemann


#endif  // MDV_RIEMANN_HYPERSHPERE_HPP
