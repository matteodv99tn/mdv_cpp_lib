#ifndef MDV_RIEMANN_TYPE_EUCLIDEAN_EMBEDDINGS_HPP
#define MDV_RIEMANN_TYPE_EUCLIDEAN_EMBEDDINGS_HPP

#include "mdv/riemann_geometry/concepts.hpp"
#include "mdv/riemann_geometry/fwd.hpp"

namespace mdv::riemann {

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


}  // namespace mdv::riemann


#endif  // MDV_RIEMANN_TYPE_EUCLIDEAN_EMBEDDINGS_HPP
