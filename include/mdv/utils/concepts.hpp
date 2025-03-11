#ifndef MDV_CONCEPTS_HPP
#define MDV_CONCEPTS_HPP

#include <Eigen/Core>

namespace mdv::concepts {


template <typename T>
concept eigen_vector = requires(T t) {
    requires std::is_base_of_v<Eigen::MatrixBase<T>, T>;
    requires T::ColsAtCompileTime == 1 || T::RowsAtCompileTime == 1;
};

template <typename T>
concept eigen_vector_expression = requires(T t) {
    // Eigen expressions must inherit from Eigen::EigenBase
    requires std::is_base_of_v<Eigen::EigenBase<T>, T>;

    // The evaluated result of the expression must be a vector
    requires eigen_vector<typename T::PlainObject>;
};

template <typename T>
concept eigen_vector_like = eigen_vector<T> || eigen_vector_expression<T>;


}  // namespace mdv::concepts


#endif  // MDV_CONCEPTS_HPP
