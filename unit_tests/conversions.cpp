#include "mdv/utils/conversions.hpp"

#include <gtest/gtest.h>

TEST(Conversions, ToSecond) {
    using mdv::convert::seconds;
    using namespace std::chrono_literals;

    ASSERT_DOUBLE_EQ(seconds(1s), 1.0);
    ASSERT_DOUBLE_EQ(seconds(1ms), 1e-3);
    ASSERT_DOUBLE_EQ(seconds(1us), 1e-6);
    ASSERT_DOUBLE_EQ(seconds(1ns), 1e-9);
}
