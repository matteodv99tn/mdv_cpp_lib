#include "mdv/utils/logging.hpp"

#include <algorithm>
#include <fmt/format.h>
#include <set>
#include <spdlog/common.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks-inl.h>
#include <spdlog/spdlog.h>

#include "mdv/utils/spdlog.hpp"


//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

using mdv::LoggerPtr;

std::string format_logger_name(const std::string& name);

//  ___                 _                           _        _   _
// |_ _|_ __ ___  _ __ | | ___ _ __ ___   ___ _ __ | |_ __ _| |_(_) ___  _ __
//  | || '_ ` _ \| '_ \| |/ _ \ '_ ` _ \ / _ \ '_ \| __/ _` | __| |/ _ \| '_ \
//  | || | | | | | |_) | |  __/ | | | | |  __/ | | | || (_| | |_| | (_) | | | |
// |___|_| |_| |_| .__/|_|\___|_| |_| |_|\___|_| |_|\__\__,_|\__|_|\___/|_| |_|
//               |_|
LoggerPtr
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

LoggerPtr
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
