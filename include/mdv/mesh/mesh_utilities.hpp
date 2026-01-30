#ifndef MDV_MESH_UTILITIES_HPP
#define MDV_MESH_UTILITIES_HPP

#include <Eigen/Dense>
#include <filesystem>
#include <functional>

namespace mdv::mesh {


/**
 * @brief Sampling parameters for function-defined meshes.
 */
struct DrawableFunctionParameters {
    using Range = std::pair<double, double>;

    // Range configurations
    Range       x_range                = {-1.0, 1.0};
    Range       y_range                = {-1.0, 1.0};
    std::size_t x_discretisation_steps = 30;
    std::size_t y_discretisation_steps = 30;
};

/**
 * @brief Creates a closed mesh from a height function and writes it to disk.
 *
 * @param destination Output file path.
 * @param f Height function z = f(x, y).
 * @param plane_z_coord Z coordinate of the closing plane.
 * @param parameters Sampling parameters.
 */
void create_closed_from_function(
        const std::filesystem::path&          destination,
        std::function<double(double, double)> f,
        double                                plane_z_coord,
        const DrawableFunctionParameters&     parameters = DrawableFunctionParameters()
);

/**
 * @brief Creates a closed mesh from a height function and returns its path.
 *
 * @param f Height function z = f(x, y).
 * @param plane_z_coord Z coordinate of the closing plane.
 * @param parameters Sampling parameters.
 * @return Output file path.
 */
std::filesystem::path create_closed_from_function(
        std::function<double(double, double)> f,
        double                                plane_z_coord,
        const DrawableFunctionParameters&     parameters = DrawableFunctionParameters()
);

/**
 * @brief Creates an open surface mesh from a height function and writes it.
 *
 * @param destination Output file path.
 * @param f Height function z = f(x, y).
 * @param parameters Sampling parameters.
 */
void create_from_function(
        const std::filesystem::path&          destination,
        std::function<double(double, double)> f,
        const DrawableFunctionParameters&     parameters = DrawableFunctionParameters()
);

/**
 * @brief Creates an open surface mesh from a height function and writes it.
 *
 * @param destination Output file path.
 * @param f Height function z = f(x, y).
 * @param parameters Sampling parameters.
 */
void create_from_function(
        const std::filesystem::path&                  destination,
        std::function<double(const Eigen::Vector2d&)> f,
        const DrawableFunctionParameters& parameters = DrawableFunctionParameters()
);

/**
 * @brief Creates an open surface mesh from a height function and returns its path.
 *
 * @param f Height function z = f(x, y).
 * @param parameters Sampling parameters.
 * @return Output file path.
 */
std::filesystem::path create_from_function(
        std::function<double(double, double)> f,
        const DrawableFunctionParameters&     parameters = DrawableFunctionParameters()
);

/**
 * @brief Creates an open surface mesh from a height function and returns its path.
 *
 * @param f Height function z = f(x, y).
 * @param parameters Sampling parameters.
 * @return Output file path.
 */
std::filesystem::path create_from_function(
        std::function<double(const Eigen::Vector2d&)> f,
        const DrawableFunctionParameters& parameters = DrawableFunctionParameters()
);


/**
 * @brief Creates a saddle mesh and returns the output path.
 *
 * @return Output file path.
 */
std::filesystem::path create_saddle();

/**
 * @brief Creates a cone mesh and returns the output path.
 *
 * @param n_edges Number of cone edges.
 * @param angle_from_axis_deg Cone angle from the axis in degrees.
 * @return Output file path.
 */
std::filesystem::path create_cone(
        std::size_t n_edges = 5, double angle_from_axis_deg = 40.0
);

/**
 * @brief Creates a flat rectangular mesh and returns the output path.
 *
 * @param size Side length of the square patch.
 * @return Output file path.
 */
std::filesystem::path create_flat(double size);

}  // namespace mdv::mesh


#endif  // MDV_MESH_UTILITIES_HPP
