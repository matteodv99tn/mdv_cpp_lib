#ifndef MDV_MESH_UTILITIES_HPP
#define MDV_MESH_UTILITIES_HPP

#include <Eigen/Dense>
#include <filesystem>
#include <functional>

namespace mdv::mesh {


struct DrawableFunctionParameters {
    using Range = std::pair<double, double>;

    // Range configurations
    Range       x_range                = {-1.0, 1.0};
    Range       y_range                = {-1.0, 1.0};
    std::size_t x_discretisation_steps = 30;
    std::size_t y_discretisation_steps = 30;
};

void create_from_function(
        const std::filesystem::path&          destination,
        std::function<double(double, double)> f,
        const DrawableFunctionParameters&     parameters = DrawableFunctionParameters()
);

void create_from_function(
        const std::filesystem::path&                  destination,
        std::function<double(const Eigen::Vector2d&)> f,
        const DrawableFunctionParameters& parameters = DrawableFunctionParameters()
);

std::filesystem::path create_from_function(
        std::function<double(double, double)> f,
        const DrawableFunctionParameters&     parameters = DrawableFunctionParameters()
);

std::filesystem::path create_from_function(
        std::function<double(const Eigen::Vector2d&)> f,
        const DrawableFunctionParameters& parameters = DrawableFunctionParameters()
);


/**
 * @brief Creates a saddle mesh and returns the path where the file has been created
 *
 */
std::filesystem::path create_saddle();

/**
 * @brief Creates a cone mesh and returns the path where the file has been created
 *
 */
std::filesystem::path create_cone(
        std::size_t n_edges = 5, double angle_from_axis_deg = 40.0
);

}  // namespace mdv::mesh


#endif  // MDV_MESH_UTILITIES_HPP
