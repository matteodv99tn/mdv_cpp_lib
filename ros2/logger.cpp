#include "mdv/ros2/logger.hpp"

#include <rclcpp/logging.hpp>

using mdv::ros2::RosLogger;

void
RosLogger::set_log_level(LogLevel level) {
    _level = level;
    switch (level) {
        case Logger::Trace:
        case Logger::Debug:
            _logger.set_level(rclcpp::Logger::Level::Debug);
            break;
        case Logger::Info:
            _logger.set_level(rclcpp::Logger::Level::Info);
            break;
        case Logger::Warning:
            _logger.set_level(rclcpp::Logger::Level::Warn);
            break;
        case Logger::Error:
            _logger.set_level(rclcpp::Logger::Level::Error);
            break;
    }
}

mdv::Logger::LogLevel
RosLogger::get_log_level() const {
    return _level;
}

void
RosLogger::log_message(std::string msg, LogLevel level) const {
    if(level < _level) return;

    switch (level) {
        case Logger::Trace:
        case Logger::Debug:
            RCLCPP_DEBUG(_logger, "%s", msg.c_str());
            break;
        case Logger::Info:
            RCLCPP_INFO(_logger, "%s", msg.c_str());
            break;
        case Logger::Warning:
            RCLCPP_WARN(_logger, "%s", msg.c_str());
            break;
        case Logger::Error:
            RCLCPP_ERROR(_logger, "%s", msg.c_str());
            break;
    }
}
