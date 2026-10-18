#include "mdv/utils/spdlog.hpp"

#include <algorithm>
#include <set>
#include <spdlog/common.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks-inl.h>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace {
std::string
format_logger_name(const std::string& name) {
    std::string          formatted_name;
    const std::set<char> undesired_chars{' ', '_', ',', '.'};
    std::transform(
            name.begin(),
            name.end(),
            std::back_inserter(formatted_name),
            [undesired_chars](char c) {
                const char c_low = std::tolower(c);  // NOLINT: narrowing conversion
                if (undesired_chars.find(c_low) != undesired_chars.end()) return '-';
                return c_low;
            }
    );
    return formatted_name;
}
}  // namespace

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

mdv::Logger::SharedPtr
mdv::class_logger_factory(
        const std::string&      class_name,
        const std::string&      instance_name,
        const Logger::LogLevel& level
) {
    using SpdLoggerPtr = std::shared_ptr<spdlog::logger>;
    const std::string logger_name =
            format_logger_name(class_name + "-" + instance_name);
    SpdLoggerPtr logger = spdlog::get(logger_name);
    if (!logger) logger = spdlog::stdout_color_st(logger_name);

    const std::string fmt = fmt::format("[%l][{}][{}] %v", class_name, instance_name);
    logger->set_pattern(fmt);

    auto res = std::make_shared<SpdLogger>(std::move(logger), level);
    return res;
}

mdv::Logger::SharedPtr
mdv::static_logger_factory(
        const std::string& logger_name, const Logger::LogLevel& level
) {
    using SpdLoggerPtr  = std::shared_ptr<spdlog::logger>;
    SpdLoggerPtr logger = spdlog::get(logger_name);
    if (!logger) logger = spdlog::stdout_color_st(logger_name);

    const std::string fmt = fmt::format("[%l][{}] %v", logger_name);
    logger->set_pattern(fmt);

    return std::make_shared<SpdLogger>(std::move(logger), level);
}
