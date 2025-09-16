#ifndef MDV_LOGGING_HPP
#define MDV_LOGGING_HPP

#include <cstdint>
#include <Eigen/Dense>
#include <fmt/format.h>
#include <memory>
#include <string>

#include "mdv/macros.hpp"

namespace mdv {

class Logger {
public:
    using UniquePtr = std::unique_ptr<Logger>;
    using SharedPtr = std::shared_ptr<Logger>;

    virtual ~Logger() = default;

    enum LogLevel : std::uint8_t {
        Trace = 0,
        Debug,
        Info,
        Warning,
        Error
    };

    virtual void                   set_log_level(LogLevel lev) = 0;
    MDV_NODISCARD virtual LogLevel get_log_level() const       = 0;

#ifdef REGISTER_LOG_LEVEL
#undef REGISTER_LOG_LEVEL
#endif

#if FMT_VERSION >= 80000
#define REGISTER_LOG_LEVEL(function, level_enum)                                       \
    template <typename... Args>                                                        \
    void function(fmt::format_string<const Args&...> fmt_str, const Args&... args)     \
            const {                                                                    \
        log_message(                                                                   \
                fmt::format(fmt_str, std::forward<const Args&>(args)...), level_enum   \
        );                                                                             \
    }
#else
#define REGISTER_LOG_LEVEL(function, level_enum)                                       \
    template <typename... Args>                                                        \
    void function(fmt::string_view fmt_str, const Args&... args) const {               \
        constexpr std::size_t n_args = sizeof...(Args);                                \
        if constexpr (n_args == 0) {                                                   \
            log_message(fmt::format(fmt_str), level_enum);                             \
        } else {                                                                       \
            using output_it_t = std::back_insert_iterator<std::string>;                \
            log_message(                                                               \
                    fmt::vformat(fmt_str, fmt::make_format_args(args...)), level_enum  \
            );                                                                         \
        }                                                                              \
    }
#endif

    REGISTER_LOG_LEVEL(trace, Trace);
    REGISTER_LOG_LEVEL(debug, Debug);
    REGISTER_LOG_LEVEL(info, Info);
    REGISTER_LOG_LEVEL(warn, Warning);
    REGISTER_LOG_LEVEL(warning, Warning);
    REGISTER_LOG_LEVEL(error, Error);

private:
    virtual void log_message(std::string msg, LogLevel lvl) const = 0;

#undef REGISTER_LOG_LEVEL
};

Logger::SharedPtr get_default_logger();

void set_default_logger(Logger::SharedPtr logger);

Logger::SharedPtr get_void_logger();

}  // namespace mdv


#endif  // MDV_LOGGING_HPP
