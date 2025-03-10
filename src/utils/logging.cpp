#include "mdv/utils/logging.hpp"

#include <algorithm>
#include <fmt/format.h>
#include <set>
#include <spdlog/common.h>
#include <spdlog/sinks/stdout_color_sinks-inl.h>
#include <spdlog/spdlog.h>


//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

using mdv::SpdLoggerPtr;

std::string format_logger_name(const std::string& name);

spdlog::level::level_enum to_spdlog_level(const mdv::LogLevel& level);

//  ___                 _                           _        _   _
// |_ _|_ __ ___  _ __ | | ___ _ __ ___   ___ _ __ | |_ __ _| |_(_) ___  _ __
//  | || '_ ` _ \| '_ \| |/ _ \ '_ ` _ \ / _ \ '_ \| __/ _` | __| |/ _ \| '_ \
//  | || | | | | | |_) | |  __/ | | | | |  __/ | | | || (_| | |_| | (_) | | | |
// |___|_| |_| |_| .__/|_|\___|_| |_| |_|\___|_| |_|\__\__,_|\__|_|\___/|_| |_|
//               |_|
SpdLoggerPtr
mdv::class_logger_factory(
        const std::string& class_name,
        const std::string& instance_name,
        const LogLevel&    level
) {
    const std::string logger_name =
            format_logger_name(class_name + "-" + instance_name);
    SpdLoggerPtr logger = spdlog::get(logger_name);
    if (!logger) logger = spdlog::stdout_color_st(logger_name);

    const std::string fmt = fmt::format("[%l][{}][{}] %v", class_name, instance_name);
    logger->set_pattern(fmt);
    logger->set_level(to_spdlog_level(level));


    return logger;
}

SpdLoggerPtr
mdv::static_logger_factory(const std::string& logger_name, const LogLevel& level) {
    SpdLoggerPtr logger = spdlog::get(logger_name);
    if (!logger) logger = spdlog::stdout_color_st(logger_name);

    const std::string fmt = fmt::format("[%l][{}] %v", logger_name);
    logger->set_pattern(fmt);
    logger->set_level(to_spdlog_level(level));
    return logger;
}

//  ____  _        _   _        _____
// / ___|| |_ __ _| |_(_) ___  |  ___|   _ _ __   ___ ___
// \___ \| __/ _` | __| |/ __| | |_ | | | | '_ \ / __/ __|
//  ___) | || (_| | |_| | (__  |  _|| |_| | | | | (__\__ \_
// |____/ \__\__,_|\__|_|\___| |_|   \__,_|_| |_|\___|___(_)
//

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

spdlog::level::level_enum
to_spdlog_level(const mdv::LogLevel& level) {
    switch (level) {
        case mdv::LogLevel::Trace:
            return spdlog::level::trace;
        case mdv::LogLevel::Debug:
            return spdlog::level::debug;
        case mdv::LogLevel::Info:
            return spdlog::level::info;
        case mdv::LogLevel::Warn:
            return spdlog::level::warn;
        case mdv::LogLevel::Error:
            return spdlog::level::err;
        default:
            return spdlog::level::info;
    }
}
