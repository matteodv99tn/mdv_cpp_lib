#include "mdv/utils/spdlog.hpp"

#include <spdlog/common.h>
#include <spdlog/logger.h>
#include <stdexcept>

mdv::SpdLogger::SpdLogger(
        std::shared_ptr<spdlog::logger> logger, const LogLevel level
) :
        _concrete_logger(std::move(logger)) {
    if (_concrete_logger == nullptr) {
        throw std::invalid_argument(
                "Provided pointer to concrete Spdlog implementation is null"
        );
    }
    set_log_level(level);
};

void
mdv::SpdLogger::set_log_level(const LogLevel level) {
    using spdlog::level::level_enum;

    assert(_concrete_logger != nullptr);

    switch (level) {
        case Logger::Trace:
            _concrete_logger->set_level(level_enum::trace);
            break;
        case Logger::Debug:
            _concrete_logger->set_level(level_enum::debug);
            break;
        case Logger::Info:
            _concrete_logger->set_level(level_enum::info);
            break;
        case Logger::Warning:
            _concrete_logger->set_level(level_enum::warn);
            break;
        case Logger::Error:
            _concrete_logger->set_level(level_enum::err);
            break;
    }
}

mdv::Logger::LogLevel
mdv::SpdLogger::get_log_level() const {
    assert(_concrete_logger != nullptr);

    switch (_concrete_logger->level()) {
        case spdlog::level::trace:
            return LogLevel::Trace;
        case spdlog::level::debug:
            return LogLevel::Debug;
        case spdlog::level::info:
            return LogLevel::Info;
        case spdlog::level::warn:
            return LogLevel::Warning;
        case spdlog::level::err:
            return LogLevel::Error;
        case spdlog::level::critical:
        case spdlog::level::off:
        case spdlog::level::n_levels:
            throw std::runtime_error(
                    "Logging level set on Spdlog is not supported my mdv::Logger "
                    "wrapper"
            );
    }
    throw std::runtime_error(
            "Logging level set on Spdlog is not supported my mdv::Logger wrapper"
    );
}

void
mdv::SpdLogger::log_message(const std::string msg, const LogLevel lvl) const {
    switch (lvl) {
        case Logger::Trace:
            _concrete_logger->trace("{}", msg);
            break;
        case Logger::Debug:
            _concrete_logger->debug("{}", msg);
            break;
        case Logger::Info:
            _concrete_logger->info("{}", msg);
            break;
        case Logger::Warning:
            _concrete_logger->warn("{}", msg);
            break;
        case Logger::Error:
            _concrete_logger->error("{}", msg);
            break;
    }
}
