#ifndef MDV_LOGGING_HPP
#define MDV_LOGGING_HPP

#include <cstdint>
#include <Eigen/Dense>
#include <fmt/format.h>
#include <memory>
#include <spdlog/fwd.h>
#include <sstream>
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
        const std::string& logger_name, const LogLevel& level = Info
);

}  // namespace mdv

template <>
class fmt::formatter<Eigen::Quaterniond> {
public:
    constexpr auto
    parse(format_parse_context& ctx) {
        return ctx.begin();
    }

    template <typename Context>
    constexpr auto
    format(Eigen::Quaterniond const& q, Context& ctx) const {
        return format_to(ctx.out(), "({}, {}, {}, {})", q.w(), q.x(), q.y(), q.z());
    }
};

template <>
class fmt::formatter<Eigen::VectorXd> {
public:
    constexpr auto
    parse(format_parse_context& ctx) {
        return ctx.begin();
    }

    template <typename Context>
    auto
    format(Eigen::VectorXd const& v, Context& ctx) const {
        std::stringstream ss;
        ss << v.transpose();
        return format_to(ctx.out(), "{}", ss.str());
    }
};

#endif  // MDV_LOGGING_HPP
