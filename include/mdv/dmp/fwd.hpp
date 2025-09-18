#ifndef MDV_DMP_FWD_HPP
#define MDV_DMP_FWD_HPP

#include "mdv/riemann_geometry/concepts.hpp"

namespace mdv {

// template <typename T>
// struct DefaultManifoldEmbedding;

template <typename T>
struct TrivialTypeEmbedding;

template <concepts::euclidean_type T>
struct TrivialTypeEmbedding<T> {
    using Input  = T;
    using Output = T;

    Output
    embed(const Input& in) const {
        return in;
    }

    Input
    decode(const Output& out) const {
        return out;
    }
};


}  // namespace mdv


#endif  // MDV_DMP_FWD_HPP
