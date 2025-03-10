#include <gtest/gtest.h>

#include <mdv/utils/conversions.hpp>

TEST(Conversions, ToSecond) {
    using mdv::convert::seconds;
    using namespace std::chrono_literals;

    ASSERT_FLOAT_EQ(seconds(1s), 1);
    ASSERT_FLOAT_EQ(seconds(1ms), 1e-3);
    ASSERT_FLOAT_EQ(seconds(1us), 1e-6);
    ASSERT_FLOAT_EQ(seconds(1ns), 1e-9);
}
