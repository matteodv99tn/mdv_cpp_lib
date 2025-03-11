#include "mdv/utils/logging_extras.hpp"

#include <Eigen/Dense>
#include <fmt/format.h>
#include <sstream>
#include <string>

#include <range/v3/all.hpp>

#include "mdv/utils/logging.hpp"

namespace rs = ranges;
namespace rv = ranges::views;

std::string
mdv::eigen_to_str(const Eigen::VectorXd& vec) {
    std::stringstream ss;
    ss << vec.transpose();

    // Format vector string to be like (xx, xx, xx)
    const std::string original_str = ss.str();
    const auto        formatted_string =
            original_str | rv::split(' ')
            | rv::filter([](const auto& rng) { return rs::distance(rng) > 0; })
            | rv::join(", ") | rs::to<std::string>;
    return fmt::format("({})", formatted_string);
}

std::string
mdv::array3d_to_str(const std::array<double, 3>& arr) {
    return fmt::format("({}, {}, {})", arr[0], arr[1], arr[2]);
}
