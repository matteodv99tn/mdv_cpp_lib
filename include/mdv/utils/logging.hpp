#ifndef MDV_LOGGING_HPP
#define MDV_LOGGING_HPP

#include <memory>
#include <spdlog/fwd.h>
#include <string>

namespace mdv {

using LoggerPtr_t = std::shared_ptr<spdlog::logger>;

enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error
};

LoggerPtr_t class_logger_factory(
        const std::string& class_name,
        const std::string& instance_name,
        const LogLevel&    level = Info
);

LoggerPtr_t static_logger_factory(
        const std::string& class_name, const LogLevel& level = Info
);


}  // namespace mdv


#endif  // MDV_LOGGING_HPP
