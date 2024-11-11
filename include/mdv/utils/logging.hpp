#ifndef MDV_LOGGING_HPP
#define MDV_LOGGING_HPP

#include <cstdint>
#include <memory>
#include <spdlog/fwd.h>
#include <string>

namespace mdv {

using SpdLoggerPtr = std::shared_ptr<spdlog::logger>;

enum LogLevel : std::uint8_t {
    Trace,
    Debug,
    Info,
    Warn,
    Error
};

SpdLoggerPtr class_logger_factory(
        const std::string& class_name,
        const std::string& instance_name,
        const LogLevel&    level = Info
);

SpdLoggerPtr static_logger_factory(
        const std::string& class_name, const LogLevel& level = Info
);


}  // namespace mdv


#endif  // MDV_LOGGING_HPP
