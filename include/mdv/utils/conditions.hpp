#ifndef MDV_UTILS_CONDITIONAL_HPP
#define MDV_UTILS_CONDITIONAL_HPP

#include <concepts>
#include <Eigen/Dense>

#include <mdv/utils/concepts.hpp>

namespace mdv::condition {

namespace internal {
    constexpr double zero_th = 1e-6;
}  // namespace internal

bool
is_zero(const std::floating_point auto value) {
    return std::abs(value) < internal::zero_th;
}

bool
is_zero_norm(const auto v) {
    return v.norm() < internal::zero_th;
}

bool
is_unit_norm(const auto v) {
    return std::abs(v.norm() - 1.0) < internal::zero_th;
}

bool
are_equal(
        const concepts::eigen_vector_like auto& v1,
        const concepts::eigen_vector_like auto& v2
) {
    return is_zero_norm(v1 - v2);
}

bool
are_parallel(
        const concepts::eigen_vector_like auto& v1,
        const concepts::eigen_vector_like auto& v2
) {
    return is_zero(std::abs((v1.normalized()).dot(v2.normalized())) - 1.0);
}

bool
are_orthogonal(
        const concepts::eigen_vector_like auto& v1,
        const concepts::eigen_vector_like auto& v2
) {
    return is_zero(v1.dot(v2));
}


}  // namespace mdv::condition


#endif  // MDV_UTILS_CONDITIONAL_HPP
