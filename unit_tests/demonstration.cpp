#include <array>
#include <gtest/gtest.h>

#include <mdv/containers/demonstration.hpp>
#include <mdv/riemann_geometry/manifold.hpp>

using mdv::Demonstration;
using mdv::riemann::S3;
using mdv::riemann::Scalar;

TEST(Demonstration2Assertions, Demonstration) {
    static_assert(mdv::riemann::manifold<S3>);

    using row = std::tuple<
            std::chrono::steady_clock::duration,
            Eigen::Quaterniond,
            Eigen::Vector4d,
            Eigen::Vector4d>;
    static_assert(std::is_same_v<row, Demonstration<S3>::BaseTuple>);

    const Eigen::Quaterniond q  = Eigen::Quaterniond::UnitRandom();
    const Eigen::Vector4d    v1 = Eigen::Vector4d::Random();
    const Eigen::Vector4d    v2 = Eigen::Vector4d::Random();

    Demonstration<S3> demo;
    using Time = decltype(demo)::Time;
    demo.append_sample().time(Time::zero()).position(q).velocity(v1).acceleration(v2);

    EXPECT_EQ(demo[0].t(), Time::zero()) << "Sample time setter";
    EXPECT_EQ(demo[0].y(), q) << "Sample position setter";
    EXPECT_EQ(demo[0].yd(), v1) << "Sample velocity setter";
    EXPECT_EQ(demo[0].ydd(), v2) << "Sample acceleration setter";
}

TEST(Demonstration2Assertions, IteratorAssertions) {
    using Dem = Demonstration<Scalar>;
    using mdv::internal::make_element_iterator;

    Dem demo;
    static_assert(std::is_same_v<Dem::Sample&, decltype(*(demo.begin()))>);
}

TEST(Demonstration2Assertions, DemonstrationBuilder) {
    using Dem = Demonstration<Scalar>;

    using namespace std::chrono_literals;


    auto dem_builder = Dem::builder();
    dem_builder.assign_position({1.0, 2.0, 3.0});
    dem_builder.fill_time(Dem::Time::zero()).fill_velocity(2.0).fill_acceleration(3.0);
    dem_builder.set_sampling_period(1ms);
    const auto dem = dem_builder.create();

    EXPECT_EQ(dem[0].y(), 1.0);
    EXPECT_EQ(dem[1].y(), 2.0);
    EXPECT_EQ(dem[2].y(), 3.0);
    EXPECT_EQ(dem[0].t(), 0ms);
    EXPECT_EQ(dem[1].t(), 1ms);
    EXPECT_EQ(dem[2].t(), 2ms);
    EXPECT_EQ(distance(dem.begin(), dem.end()), 3);

    for (const auto& sample : dem) {
        EXPECT_EQ(sample.yd(), 2.0);
        EXPECT_EQ(sample.ydd(), 3.0);
    }
}

class TestableDemonstration : public Demonstration<Scalar> {
public:
    using Demo = Demonstration<Scalar>;

    using Sample = Demo::Sample;

    TestableDemonstration() {
        using mdv::internal::make_element_iterator;
        static_assert(std::is_same_v<Sample&, decltype(*begin())>);
        auto base_it = begin();
        static_assert(std::is_same_v<decltype(base_it)::value_type, Sample>);

        auto it = make_element_iterator<1>(begin());
        static_assert(std::is_same_v<double&, decltype(*it)>);
    }
};

template <typename Demo, typename M>
// template <typename M, typename Demo>
concept is_demonstration = requires {
    requires mdv::demonstration_manifold_type<M>;
    requires std::is_same_v<M, typename Demo::Manifold>;
};


// static_assert(is_demonstration<Scalar> Demonstration<Scalar>);

template <mdv::manifold_demonstration<Scalar> Demo>
static void process_demo(Demo& dem) {};

int
main() {
    Demonstration<Scalar> d;
    process_demo(d);
}
