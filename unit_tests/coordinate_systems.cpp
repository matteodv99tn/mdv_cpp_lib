#include <catch2/catch_test_macros.hpp>
#include <mdv/dmp/coordinate_system/coordinate_system.hpp>

TEST_CASE("Testing base coordinate system class", "[dmp][coordinate system]") {
    /*
     * Create a "dummy" coordinate system class which uses the time scaling factor tau
     * to multiply the provided type, and then verify the outputs.
     */
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

    double               tau = 2.0;
    FakeCoordinateSystem cs(&tau);

    REQUIRE(cs(1.0) == 2.0);
    tau = 3.0;
    REQUIRE(cs(1.0) == 3.0);

    const Eigen::VectorXd ts_eigen = Eigen::VectorXd::LinSpaced(100, 0.0, 10.0);
    std::vector<double>   ts_vec_in;
    std::vector<double>   ts_vec_out;
    for (const auto& e : ts_eigen) {
        ts_vec_in.push_back(e);
        ts_vec_out.push_back(e * tau);
    }

    REQUIRE(cs(ts_eigen) == tau * ts_eigen);
    REQUIRE(cs(ts_vec_in) == ts_vec_out);
}
