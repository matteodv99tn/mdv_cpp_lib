#ifndef MDV_CONVERSION_UTILITIES_HPP
#define MDV_CONVERSION_UTILITIES_HPP

#include <chrono>

namespace mdv::convert {

template <typename T>
double seconds(const T&);

template <class Rep, class Period>
double
seconds(const std::chrono::duration<Rep, Period>& t) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(t).count();
}

template <>
inline double
seconds<double>(const double& t) {
    return t;
}


}  // namespace mdv::convert


#endif  // MDV_CONVERSION_UTILITIES_HPP
