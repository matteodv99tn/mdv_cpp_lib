#ifndef MDV_ROS2_LOGGER_INTERFACE_HPP
#define MDV_ROS2_LOGGER_INTERFACE_HPP

#include <rclcpp/logger.hpp>

#include "mdv/utils/logging.hpp"

namespace mdv::ros2 {

class RosLogger : public Logger {
public:
    RosLogger(rclcpp::Logger logger) : _logger(logger){};
    ~RosLogger() override = default;

    void                   set_log_level(LogLevel level) override;
    MDV_NODISCARD LogLevel get_log_level() const override;
    void                   log_message(std::string msg, LogLevel level) const override;

private:
    rclcpp::Logger   _logger;
    Logger::LogLevel _level = LogLevel::Info;
};


}  // namespace mdv::ros2


#endif  // MDV_ROS2_LOGGER_INTERFACE_HPP
