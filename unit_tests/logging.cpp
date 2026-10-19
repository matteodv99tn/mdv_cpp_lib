#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "mdv/utils/spdlog.hpp"

TEST(SpdLogger, NormalizesClassAndInstanceNames) {
    const auto logger =
            mdv::class_logger_factory("CI Logger_Class", "Instance,Name.Test");

    EXPECT_NE(logger, nullptr);
    EXPECT_NE(spdlog::get("ci-logger-class-instance-name-test"), nullptr);
    spdlog::drop("ci-logger-class-instance-name-test");
}
