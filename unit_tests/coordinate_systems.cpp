#include <gtest/gtest.h>
#include <mdv/dmp/coordinate_system/coordinate_system.hpp>

class FakeCoordinateSystem
        : public mdv::dmp::CoordinateSystemBase<FakeCoordinateSystem> {
    using Base = mdv::dmp::CoordinateSystemBase<FakeCoordinateSystem>;
    friend Base;

    double
    eval(const double t) const {
        return t * tau();
    }

public:
    FakeCoordinateSystem(const double* tau) : Base(tau) {}
};

TEST(CoordinateSystemTest, TestingBaseCoordinateSystemClass) {
    double               tau = 2.0;
    FakeCoordinateSystem cs(&tau);

    EXPECT_DOUBLE_EQ(cs(1.0), 2.0);
    tau = 3.0;
    EXPECT_DOUBLE_EQ(cs(1.0), 3.0);

    const Eigen::VectorXd ts_eigen = Eigen::VectorXd::LinSpaced(100, 0.0, 10.0);
    std::vector<double>   ts_vec_in;
    std::vector<double>   ts_vec_out;
    for (const auto& e : ts_eigen) {
        ts_vec_in.push_back(e);
        ts_vec_out.push_back(e * tau);
    }

    EXPECT_EQ(cs(ts_eigen), tau * ts_eigen);
    EXPECT_EQ(cs(ts_vec_in), ts_vec_out);
}

