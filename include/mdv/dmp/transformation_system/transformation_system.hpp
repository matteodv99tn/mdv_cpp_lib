#ifndef MDV_DMP_TRANSFORMATION_SYSTEM_INTERFACE_HPP
#define MDV_DMP_TRANSFORMATION_SYSTEM_INTERFACE_HPP

#include <mdv/riemann_geometry/manifold.hpp>

namespace mdv::dmp {

template <typename M, template <typename> class TsImplementation>
class TransformationSystemBase {
    using TransfSys = TsImplementation<M>;

    TransfSys&
    tf() {
        return static_cast<TransfSys&>(*this);
    };

    const TransfSys&
    tf() const {
        return static_cast<const TransfSys&>(*this);
    };

public:
    using Type    = M::Type;
    using Tangent = M::Tangent;

    Tangent
    learn(const Type& y, const Tangent& yd, const Type& g) const {
        return tf().learn_impl(y, yd, g);
    }

    std::vector<Tangent>
    learn(const std::vector<Type>&    y,
          const std::vector<Tangent>& yd,
          const std::vector<Type>&    g) const {
        std::vector<Tangent> res;
        assert(y.size() == yd.size());
        assert(y.size() == g.size());
        res.reserve(y.size());
        for (std::size_t i{0}; i < y.size(); ++i)
            res.emplace_back(learn(y[i], yd[g], g[i]));
        return res;
    }
};

template <typename M>
class SecondOrderSystem : public TransformationSystemBase<M, SecondOrderSystem> {
    using Parent = TransformationSystemBase<M, SecondOrderSystem>;
    friend Parent;

    using Type    = M::Type;
    using Tangent = M::Tangent;

    Tangent
    learn_impl(const Type& y, const Tangent& yd, const Type& g) const {
        return Tangent();
    }
};


}  // namespace mdv::dmp


#endif  // MDV_DMP_TRANSFORMATION_SYSTEM_INTERFACE_HPP
