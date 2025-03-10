#ifndef MDV_DEMONSTRATION_HPP
#define MDV_DEMONSTRATION_HPP

#include <chrono>
#include <concepts>
#include <fmt/format.h>
#include <functional>
#include <tuple>

#include "mdv/macros.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/utils/conversions.hpp"

namespace mdv {

template <typename M>
concept demonstration_manifold_type = requires {
    typename M::Point;
    typename M::TangentVector;

    requires std::default_initializable<typename M::Point>;
    requires std::default_initializable<typename M::TangentVector>;
};

template <typename Demo, typename M>
concept manifold_demonstration = requires {
    requires demonstration_manifold_type<M>;
    requires std::is_same_v<M, typename Demo::Manifold>;
};

namespace internal {
    template <typename TTangentVector, std::size_t NTangentVector, typename... Args>
    struct sample;

    template <std::size_t Index, typename BaseIterator>
    struct DemonstrationElementIterator;

    template <std::size_t Index, typename BaseIter>
    DemonstrationElementIterator<Index, BaseIter> make_element_iterator(BaseIter);

    template <typename T>
    struct tuple_lazy_function;

    template <typename T>
    using tuple_lazy_function_t = typename tuple_lazy_function<T>::type;

}  // namespace internal

template <
        demonstration_manifold_type M,
        std::size_t                 Order = 2,
        typename ClockT                   = std::chrono::steady_clock::duration>
class Demonstration {
public:
    class Sample;
    class SampleBuilder;
    class DemonstrationBuilder;

    friend class DemonstrationBuilder;

    MDV_MANIFOLD_TYPENAMES_IMPORT(M);
    using Manifold  = M;
    using Time      = ClockT;
    using BaseTuple = internal::sample<TangentVector, Order, Time, Point>::type;
    using Container = std::vector<Sample>;

    MDV_NODISCARD static Demonstration
    empty(const std::size_t size) {
        Demonstration res;
        res._data.reserve(size);
        return res;
    }

    MDV_NODISCARD static DemonstrationBuilder
    builder(std::size_t initial_size = 0) {
        return DemonstrationBuilder(initial_size);
    }

    SampleBuilder
    append_sample() {
        _data.push_back(BaseTuple());
        return SampleBuilder(&(_data.back()));
    }

    // clang-format off
    MDV_NODISCARD std::size_t size() const noexcept { return _data.size(); }

    MDV_NODISCARD Sample&       operator[](const std::size_t i)       { return _data[i]; }
    MDV_NODISCARD const Sample& operator[](const std::size_t i) const { return _data[i]; }

    MDV_NODISCARD Sample&       front()       { if(_data.size() == 0) throw std::out_of_range("Can't call front() on empty Demonstration"); return _data[0]; }
    MDV_NODISCARD const Sample& front() const { if(_data.size() == 0) throw std::out_of_range("Can't call front() on empty Demonstration"); return _data[0]; }
    MDV_NODISCARD Sample&       back()        { if(_data.size() == 0) throw std::out_of_range("Can't call back() on empty Demonstration");  return _data[_data.size() - 1]; }
    MDV_NODISCARD const Sample& back() const  { if(_data.size() == 0) throw std::out_of_range("Can't call back() on empty Demonstration");  return _data[_data.size() - 1]; }

    MDV_NODISCARD auto begin() noexcept       { return _data.begin();  }
    MDV_NODISCARD auto end() noexcept         { return _data.end();    }
    MDV_NODISCARD auto begin() const noexcept { return _data.begin();  }
    MDV_NODISCARD auto end() const noexcept   { return _data.end();    }
    MDV_NODISCARD auto cbegin() noexcept      { return _data.cbegin(); }
    MDV_NODISCARD auto cend() noexcept        { return _data.cend();   }

    // clang-format on

    std::vector<Point>
    get_position_vector() const {
        return construct_data_vector<1>();
    };

private:
    Container _data;

    template <size_t Index>
    std::vector<std::tuple_element_t<Index, BaseTuple>>
    construct_data_vector() const {
        std::vector<std::tuple_element_t<Index, BaseTuple>> res;
        res.reserve(_data.size());
        for (const auto& sample : _data)
            res.push_back(sample.template get_element<Index>());
        return res;
    }
};

//  ____                        _
// / ___|  __ _ _ __ ___  _ __ | | ___
// \___ \ / _` | '_ ` _ \| '_ \| |/ _ \
//  ___) | (_| | | | | | | |_) | |  __/
// |____/ \__,_|_| |_| |_| .__/|_|\___|
//                       |_|
template <demonstration_manifold_type M, std::size_t Order, typename ClockT>
class Demonstration<M, Order, ClockT>::Sample {
public:
    using BaseTuple = Demonstration<M, Order, ClockT>::BaseTuple;

    template <typename... Args>
    Sample(Args... args) : _data(std::forward<Args>(args)...) {}

    Sample(BaseTuple tpl) : _data(std::move(tpl)) {}

    Sample(BaseTuple& tpl) : _data(tpl) {}

    // clang-format off
    MDV_NODISCARD Time&                t()         { return get_element<0>(); }
    MDV_NODISCARD const Time&          t() const   { return get_element<0>(); }
    MDV_NODISCARD Point&               y()         { return get_element<1>(); }
    MDV_NODISCARD const Point&         y() const   { return get_element<1>(); }
    MDV_NODISCARD TangentVector&       yd()        { return get_element<2>(); static_assert(Order >= 1, "Demonstration order must be >= 1 to call yd() on a sample");  }
    MDV_NODISCARD const TangentVector& yd() const  { return get_element<2>(); static_assert(Order >= 1, "Demonstration order must be >= 1 to call yd() on a sample");  }
    MDV_NODISCARD TangentVector&       ydd()       { return get_element<3>(); static_assert(Order >= 2, "Demonstration order must be >= 2 to call ydd() on a sample"); }
    MDV_NODISCARD const TangentVector& ydd() const { return get_element<3>(); static_assert(Order >= 2, "Demonstration order must be >= 2 to call ydd() on a sample"); }

    template <std::size_t Index>
    MDV_NODISCARD auto& get_element() { return std::get<Index>(_data); }

    template <std::size_t Index>
    MDV_NODISCARD const auto& get_element() const { return std::get<Index>(_data); }

    // clang-format on
private:
    BaseTuple _data;
};

//  ____                        _      ____        _ _     _
// / ___|  __ _ _ __ ___  _ __ | | ___| __ ) _   _(_) | __| | ___ _ __
// \___ \ / _` | '_ ` _ \| '_ \| |/ _ \  _ \| | | | | |/ _` |/ _ \ '__|
//  ___) | (_| | | | | | | |_) | |  __/ |_) | |_| | | | (_| |  __/ |
// |____/ \__,_|_| |_| |_| .__/|_|\___|____/ \__,_|_|_|\__,_|\___|_|
//                       |_|
template <demonstration_manifold_type M, std::size_t Order, typename ClockT>
class Demonstration<M, Order, ClockT>::SampleBuilder {
public:
    explicit SampleBuilder(Sample* s) : _sample(s) {}

    SampleBuilder(const SampleBuilder&)            = delete;
    SampleBuilder(SampleBuilder&&)                 = delete;
    SampleBuilder& operator=(const SampleBuilder&) = delete;
    SampleBuilder& operator=(SampleBuilder&&)      = delete;

    // clang-format off
    SampleBuilder&& time(const Time& t) &&                   { _sample->t() = t;              return std::move(*this); }
    SampleBuilder&& time(Time&& t) &&                        { _sample->t() = std::move(t);   return std::move(*this); }
    SampleBuilder&& position(const Point& y) &&              { _sample->y() = y;              return std::move(*this); }
    SampleBuilder&& position(Point&& y) &&                   { _sample->y() = std::move(y);   return std::move(*this); }
    SampleBuilder&& velocity(const TangentVector& v) &&      { _sample->yd() = v;             return std::move(*this); }
    SampleBuilder&& velocity(TangentVector&& v) &&           { _sample->yd() = std::move(v);  return std::move(*this); }
    SampleBuilder&& acceleration(const TangentVector& a) &&  { _sample->ydd() = a;            return std::move(*this); }
    SampleBuilder&& acceleration(TangentVector&& a) &&       { _sample->ydd() = std::move(a); return std::move(*this); }

    // clang-format on

private:
    Sample* _sample;
};

//  ____                                 _             _   _
// |  _ \  ___ _ __ ___   ___  _ __  ___| |_ _ __ __ _| |_(_) ___  _ __
// | | | |/ _ \ '_ ` _ \ / _ \| '_ \/ __| __| '__/ _` | __| |/ _ \| '_ \
// | |_| |  __/ | | | | | (_) | | | \__ \ |_| | | (_| | |_| | (_) | | | |
// |____/ \___|_| |_| |_|\___/|_| |_|___/\__|_|  \__,_|\__|_|\___/|_| |_|
//
//  ____        _ _     _
// | __ ) _   _(_) | __| | ___ _ __
// |  _ \| | | | | |/ _` |/ _ \ '__|
// | |_) | |_| | | | (_| |  __/ |
// |____/ \__,_|_|_|\__,_|\___|_|
//
template <demonstration_manifold_type M, std::size_t Order, typename ClockT>
class Demonstration<M, Order, ClockT>::DemonstrationBuilder {
    using Dem = Demonstration<M, Order, ClockT>;

public:
    DemonstrationBuilder(std::size_t size = 0) { data().reserve(size); }

    Dem
    create() {
        using mdv::convert::seconds;
        evaluate_all_lazy_functions(std::make_index_sequence<Order + 2>());

        const std::size_t data_size = data().size();
        if (_vel_autodiff) {
            for (long i = 0; i < data_size - 1; ++i) {
                auto&        curr = data(i);
                const auto&  next = data(i + 1);
                const double dt   = seconds(next.t() - curr.t());
                curr.yd()         = M::logarithmic_map(curr.y(), next.y()) / dt;
            }
            auto&       last_sample       = data(data_size - 1);
            const auto& secondlast_sample = data(data_size - 2);

            last_sample.yd() =
                    M::covariant_derivative(last_sample.y(), secondlast_sample.yd());
        }

        if (_acc_autodiff) {
            for (long i = 0; i < data_size - 1; ++i) {
                auto&        curr    = data(i);
                const auto&  next    = data(i + 1);
                const auto&  next_y  = next.y();
                const auto&  next_yd = next.yd();
                const auto&  curr_y  = curr.y();
                const auto&  curr_yd = curr.yd();
                const double dt      = seconds(next.t() - curr.t());
                const auto&  yd_next_transp =
                        M::parallel_transport(next_y, curr_y, next_yd);
                curr.ydd() = (yd_next_transp - curr.yd()) / dt;
            }
            auto&       last_sample       = data(data_size - 1);
            const auto& secondlast_sample = data(data_size - 2);
            last_sample.ydd() =
                    M::covariant_derivative(last_sample.y(), secondlast_sample.ydd());
        }
        return std::move(_dem);
    }

    DemonstrationBuilder&
    assign_time(const std::vector<Time>& data) {
        assign_vector<0>(data, "time");
        return *this;
    }

    DemonstrationBuilder&
    assign_position(const std::vector<Point>& data) {
        assign_vector<1, Point>(data, "position");
        return *this;
    }

    DemonstrationBuilder&
    assign_velocity(const std::vector<TangentVector>& data) {
        assign_vector<2, TangentVector>(data, "velocity");
        return *this;
    }

    DemonstrationBuilder&
    assign_acceleration(const std::vector<TangentVector>& data) {
        assign_vector<3, TangentVector>(data, "acceleration");
        return *this;
    }

    DemonstrationBuilder&
    set_sampling_period(const Time& ts) {
        std::get<0>(_lazy_funcs) = [ts](std::size_t i) { return ts * i; };
        return *this;
    }

    DemonstrationBuilder&
    fill_time(const Time& value) {
        constant_lazy_function<0>(value);
        return *this;
    }

    DemonstrationBuilder&
    fill_position(const Point& value) {
        constant_lazy_function<1>(value);
        return *this;
    }

    DemonstrationBuilder&
    fill_velocity(const TangentVector& value) {
        constant_lazy_function<2>(value);
        return *this;
    }

    DemonstrationBuilder&
    fill_acceleration(const TangentVector& value) {
        constant_lazy_function<3>(value);
        return *this;
    }

    DemonstrationBuilder&
    velocity_automatic_differentiation() {
        _vel_autodiff = true;
        return *this;
    }

    DemonstrationBuilder&
    acceleration_automatic_differentiation() {
        _acc_autodiff = true;
        return *this;
    }


protected:
    // clang-format off
    MDV_NODISCARD Dem::Container&       data()                          { return _dem._data; }
    MDV_NODISCARD const Dem::Container& data() const                    { return _dem._data; }
    MDV_NODISCARD Dem::Sample&          data(const std::size_t i)       { return _dem._data[i]; }
    MDV_NODISCARD const Dem::Sample&    data(const std::size_t i) const { return _dem._data[i]; }

    // clang-format on

private:
    mutable Demonstration                      _dem;
    internal::tuple_lazy_function_t<BaseTuple> _lazy_funcs;
    bool                                       _vel_autodiff = false;
    bool                                       _acc_autodiff = false;

    template <std::size_t Index>
    using ElementIterator =
            internal::DemonstrationElementIterator<Index, typename Container::iterator>;

    template <std::size_t Index>
    void
    lazy_eval() {
        auto& fun = std::get<Index>(_lazy_funcs);
        if (!fun) return;

        for (std::size_t i{0}; i < data().size(); ++i)
            data()[i].template get_element<Index>() = fun(i);
    }

    template <std::size_t... Indices>
    void
    evaluate_all_lazy_functions(std::index_sequence<Indices...> /* unused */) {
        (lazy_eval<Indices>(), ...);
    }

    template <std::size_t Index>
    void
    constant_lazy_function(const std::tuple_element_t<Index, BaseTuple>& elem) {
        std::get<Index>(_lazy_funcs) = [elem](std::size_t) { return elem; };
    }

    template <std::size_t Index, typename T>
    void
    assign_vector(const std::vector<T>& content, std::string_view comp_desc) {
        if (data().size() == 0) data().resize(content.size());

        if (data().size() != content.size()) {
            throw std::runtime_error(fmt::format(
                    "Current demonstration samples count ({}) is not compatible with "
                    "provided container size ({}) for {} component, and no default "
                    "value constructor",
                    data().size(),
                    content.size(),
                    comp_desc
            ));
        }

        auto it = internal::make_element_iterator<Index>(data().begin());
        std::copy(content.begin(), content.end(), it);
    }
};

//  ___       _                        _
// |_ _|_ __ | |_ ___ _ __ _ __   __ _| |
//  | || '_ \| __/ _ \ '__| '_ \ / _` | |
//  | || | | | ||  __/ |  | | | | (_| | |
// |___|_| |_|\__\___|_|  |_| |_|\__,_|_|
//
namespace internal {

    template <typename TTangentVector, std::size_t NTangentVector, typename... Args>
    struct sample {
        using type = typename sample<
                TTangentVector,
                NTangentVector - 1,
                Args...,
                TTangentVector>::type;
    };

    template <typename TTangentVector, typename... Args>
    struct sample<TTangentVector, 0, Args...> {
        using type = std::tuple<Args...>;
    };

    template <std::size_t Index, typename BaseIterator>
    struct DemonstrationElementIterator : BaseIterator {
        using Tpl = typename BaseIterator::value_type::BaseTuple;

        std::tuple_element_t<Index, Tpl>&
        operator*() {
            return BaseIterator::operator*().template get_element<Index>();
        }
    };

    template <std::size_t Index, typename BaseIter>
    DemonstrationElementIterator<Index, BaseIter>
    make_element_iterator(BaseIter it) {
        return DemonstrationElementIterator<Index, BaseIter>(it);
    }

    template <typename... Args>
    struct tuple_lazy_function<std::tuple<Args...>> {
        using type = std::tuple<std::function<Args(std::size_t)>...>;
    };


}  // namespace internal

}  // namespace mdv


#endif  // MDV_DEMONSTRATION_HPP
