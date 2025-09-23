#include "mdv/mesh/mesh_utilities.hpp"

#include <cassert>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <fmt/format.h>
#include <fstream>
#include <gsl/narrow>
#include <vector>

#include "mdv/config.hpp"

// \cond DOXYGEN_IGNORE
using FaceData   = std::vector<Eigen::Vector3i>;
using VertexData = std::vector<Eigen::Vector3d>;

// \endcond

namespace {

void
write_off(
        const std::filesystem::path& path,
        const VertexData&            vertices,
        const FaceData&              faces
) {
    std::ofstream file(path);
    file << "OFF\n";
    file << vertices.size() << " " << faces.size() << " 0\n";

    for (const auto& v : vertices) file << v(0) << " " << v(1) << " " << v(2) << '\n';

    for (const auto& f : faces)
        file << "3 " << f(0) << " " << f(1) << " " << f(2) << '\n';

    file.close();
}

}  // namespace

void
mdv::mesh::create_from_function(
        const std::filesystem::path&          destination,
        std::function<double(double, double)> f,
        const DrawableFunctionParameters&     parameters
) {
    const long nx             = gsl::narrow<long>(parameters.x_discretisation_steps);
    const long ny             = gsl::narrow<long>(parameters.y_discretisation_steps);
    const auto [x_min, x_max] = parameters.x_range;
    const auto [y_min, y_may] = parameters.y_range;
    const Eigen::VectorXd xs  = Eigen::VectorXd::LinSpaced(nx, x_min, x_max);
    const Eigen::VectorXd ys  = Eigen::VectorXd::LinSpaced(ny, y_min, y_may);

    VertexData vertices;
    vertices.reserve(nx * ny);
    for (const double& x : xs)
        for (const double& y : ys) vertices.emplace_back(x, y, f(x, y));

    assert(vertices.size() == nx * ny);

    FaceData   faces;
    const long n_faces = (nx - 1) * (ny - 1) * 2;
    faces.reserve(n_faces);

    for (long j = 0; j < ny - 1; ++j) {
        for (long i = 0; i < nx - 1; ++i) {
            const long id_base = j * nx + i;
            faces.emplace_back(id_base, id_base + 1, id_base + 1 + nx);
            faces.emplace_back(id_base, id_base + 1 + nx, id_base + nx);
        }
    }
    assert(faces.size() == n_faces);

    write_off(destination, vertices, faces);
}

void
mdv::mesh::create_from_function(
        const std::filesystem::path&                  destination,
        std::function<double(const Eigen::Vector2d&)> f,
        const DrawableFunctionParameters&             parameters
) {
    auto fun = [f](const double x, const double y) -> double {
        return f(Eigen::Vector2d({x, y}));
    };

    create_from_function(destination, fun, parameters);
}

std::filesystem::path
mdv::mesh::create_from_function(
        std::function<double(double, double)> f,
        const DrawableFunctionParameters&     parameters
) {
    static int mesh_id = 0;

    const std::string mesh_name = fmt::format("mesh_{}.off", mesh_id);
    ++mesh_id;
    const std::filesystem::path dest =
            std::filesystem::temp_directory_path() / mesh_name;

    create_from_function(dest, f, parameters);
    return dest;
}

std::filesystem::path
mdv::mesh::create_from_function(
        std::function<double(const Eigen::Vector2d&)> f,
        const DrawableFunctionParameters&             parameters
) {
    auto fun = [f](const double x, const double y) -> double {
        return f(Eigen::Vector2d({x, y}));
    };

    return create_from_function(fun, parameters);
}

std::filesystem::path
mdv::mesh::create_saddle() {
    const std::filesystem::path res =
            std::filesystem::temp_directory_path() / "saddle.off";

    double k   = 0.4;
    auto   fun = [k](const double x, const double y) -> double {
        return k * (x * x - y * y);
    };

    const std::vector<std::pair<double, double>> pts = {
            {0.0,  0.0 },
            {-1.0, 0.0 },
            {0.0,  1.0 },
            {1.0,  0.0 },
            {0.0,  -1.0},
    };

    VertexData vertices;
    for (const auto [x, y] : pts) vertices.emplace_back(x, y, fun(x, y));

    FaceData faces{
            {0, 1, 2},
            {0, 2, 3},
            {0, 3, 4},
            {0, 4, 1}
    };

    write_off(res, vertices, faces);
    return res;
}

std::filesystem::path
mdv::mesh::create_cone(const std::size_t n_edges, const double axis_from_angle_deg) {
    const std::string           filename = fmt::format("cone_{}.off", n_edges);
    const std::filesystem::path res = std::filesystem::temp_directory_path() / filename;

    std::vector<double> thetas;
    thetas.reserve(n_edges);
    for (std::size_t i = 0; i < n_edges; ++i)
        thetas.emplace_back(double(i) / double(n_edges) * 2 * M_PI);

    const double aperture_deg = axis_from_angle_deg;
    const double aperture_rad = aperture_deg * M_PI / 180.0;

    using Vec3 = Eigen::Vector3d;
    using Eigen::AngleAxisd;
    Vec3 edge_dir = AngleAxisd(aperture_rad, Vec3::UnitY()) * Vec3::UnitX();

    VertexData vertices;
    vertices.reserve(n_edges + 1);
    vertices.emplace_back(Eigen::Vector3d::Zero());
    for (const double theta : thetas)
        vertices.emplace_back(AngleAxisd(theta, Vec3::UnitZ()) * edge_dir);

    FaceData faces;
    for (std::size_t i = 0; i < n_edges - 1; ++i) faces.emplace_back(0, i + 1, i + 2);
    faces.emplace_back(0, n_edges, 1);

    write_off(res, vertices, faces);
    return res;
}
