#ifndef MDV_SPDLOG_WRAPPER_HPP
#define MDV_SPDLOG_WRAPPER_HPP

#include <memory>
#include <spdlog/fwd.h>

#include "mdv/utils/logging.hpp"

namespace mdv {

/**
 * @brief Wrapper around a spdlog logger
 */
class SpdLogger : public Logger {
public:
    SpdLogger(std::shared_ptr<spdlog::logger> logger, LogLevel level = LogLevel::Info);
    ~SpdLogger() override = default;


    void set_log_level(LogLevel lev) override;

    MDV_NODISCARD LogLevel get_log_level() const override;

private:
    std::shared_ptr<spdlog::logger> _concrete_logger = nullptr;

    void log_message(std::string msg, LogLevel lvl) const override;
};

}  // namespace mdv


#endif  // MDV_SPDLOG_WRAPPER_HPP
