#ifndef MDV_RIEMANN_UTILS_HPP
#define MDV_RIEMANN_UTILS_HPP

#include <Eigen/Dense>

#include "mdv/riemann_geometry/concepts.hpp"

namespace mdv::riemann {

template <concepts::euclidean_type T>
struct SpaceDimension;

template <concepts::euclidean_type T>
constexpr long space_dimension_v = SpaceDimension<T>::value;

template <>
struct SpaceDimension<double> {
    static constexpr long value = 1;
};

template <int Dim>
struct SpaceDimension<Eigen::Vector<double, Dim>> {
    static_assert(Dim != -1, "Can't define dimension of runtime-dependent space");
    static constexpr long value = Dim;
};


}  // namespace mdv::riemann


#endif  // MDV_RIEMANN_UTILS_HPP
