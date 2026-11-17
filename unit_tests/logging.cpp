#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#include <sstream>

#include "mdv/utils/spdlog.hpp"

TEST(SpdLogger, NormalizesClassAndInstanceNames) {
    const auto logger =
            mdv::class_logger_factory("CI Logger_Class", "Instance,Name.Test");

    EXPECT_NE(logger, nullptr);
    EXPECT_NE(spdlog::get("ci-logger-class-instance-name-test"), nullptr);
    spdlog::drop("ci-logger-class-instance-name-test");
}

TEST(SpdLogger, PreservesFormattedMessagesAndLogLevels) {
    std::ostringstream output;
    auto               sink = std::make_shared<spdlog::sinks::ostream_sink_st>(output);
    auto concrete = std::make_shared<spdlog::logger>("raw-message-test", sink);
    concrete->set_pattern("%l|%v");
    mdv::SpdLogger logger(concrete, mdv::Logger::Trace);

    const std::string message = "literal {} {name}";
    logger.trace("{}", message);
    logger.debug("{}", message);
    logger.info("{}", message);
    logger.warn("{}", message);
    logger.error("{}", message);
    EXPECT_EQ(
            output.str(),
            "trace|literal {} {name}\n"
            "debug|literal {} {name}\n"
            "info|literal {} {name}\n"
            "warning|literal {} {name}\n"
            "error|literal {} {name}\n"
    );

    output.str("");
    logger.set_log_level(mdv::Logger::Error);
    logger.info("{}", message);
    logger.error("{}", message);
    EXPECT_EQ(output.str(), "error|literal {} {name}\n");
}
