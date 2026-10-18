#include "mdv/mesh/mesh_utilities.hpp"

#include <cassert>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <fmt/format.h>
#include <fstream>
#include <gsl/narrow>
#include <limits>
#include <optional>
#include <vector>

#include <range/v3/all.hpp>

#include "mdv/config.hpp"
#include "mdv/mesh/mesh.hpp"

namespace rs = ::ranges;
namespace rv = ::ranges::views;

// \cond DOXYGEN_IGNORE
using FaceData   = std::vector<std::vector<long>>;
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

    for (const auto& v : vertices) file << v(0) << " " << v(1) << " " << v(2) << "\n";

    for (const auto& f : faces) {
        file << f.size();
        for (const auto i : f) file << " " << i;
        file << "\n";
    }

    file.close();
}

std::pair<VertexData, FaceData>
build_surface(
        std::function<double(double, double)>        f,
        const mdv::mesh::DrawableFunctionParameters& parameters,
        const std::optional<double>                  plane_z_coord = std::nullopt
) {
    const auto logger = mdv::mesh::Mesh::default_logger;

    const long nx             = gsl::narrow<long>(parameters.x_discretisation_steps);
    const long ny             = gsl::narrow<long>(parameters.y_discretisation_steps);
    const auto [x_min, x_max] = parameters.x_range;
    const auto [y_min, y_max] = parameters.y_range;
    const Eigen::VectorXd xs  = Eigen::VectorXd::LinSpaced(nx, x_min, x_max);
    const Eigen::VectorXd ys  = Eigen::VectorXd::LinSpaced(ny, y_min, y_max);
    logger->debug(
            "Generating surface - low corner: {:.2f}, {:.2f} - top corner: {:.2f}, "
            "{:.2f} - dimension: {}x{} vertices",
            x_min,
            y_min,
            x_max,
            y_max,
            nx,
            ny
    );

    VertexData vertices;
    vertices.reserve(nx * ny + 5);
    for (const double& y : ys)
        for (const double& x : xs) vertices.emplace_back(x, y, f(x, y));


    FaceData   faces;
    const long n_faces = (nx - 1) * (ny - 1) * 2 + 2;
    faces.reserve(n_faces);

    for (long j = 0; j < ny - 1; ++j) {
        for (long i = 0; i < nx - 1; ++i) {
            const long id_base = j * nx + i;
            faces.emplace_back(
                    std::vector<long>{id_base, id_base + 1, id_base + 1 + nx}
            );
            faces.emplace_back(
                    std::vector<long>{id_base, id_base + 1 + nx, id_base + nx}
            );
        }
    }

    if (plane_z_coord.has_value()) {
        const double z = plane_z_coord.value();

        logger->debug("Adding mesh closure surfaces - plane z coordinate: {}", z);
        using Vec3     = Eigen::Vector3d;
        using Vec3Pair = std::pair<Vec3, Vec3>;
        Vec3 min       = 1000.0 * Vec3::Identity();
        Vec3 max       = -1000.0 * Vec3::Identity();

        for (const Vec3& v : vertices) {
            min = min.cwiseMin(v);
            max = max.cwiseMax(v);
        }

        const Vec3 b1_vertex{min(0), min(1), z};
        const Vec3 b2_vertex{max(0), min(1), z};
        const Vec3 b3_vertex{max(0), max(1), z};
        const Vec3 b4_vertex{min(0), max(1), z};

        const long b1_id = vertices.size();
        vertices.emplace_back(b1_vertex);
        const long b2_id = vertices.size();
        vertices.emplace_back(b2_vertex);
        const long b3_id = vertices.size();
        vertices.emplace_back(b3_vertex);
        const long b4_id = vertices.size();
        vertices.emplace_back(b4_vertex);

        std::vector<long> f1;
        f1.push_back(b2_id);
        f1.push_back(b1_id);
        for (long i = 0; i < nx; ++i) f1.push_back(i);

        std::vector<long> f2;
        f2.push_back(b3_id);
        f2.push_back(b2_id);
        for (long i = 0; i < ny; ++i) f2.push_back(nx * (i + 1) - 1);

        std::vector<long> f3;
        f3.push_back(b4_id);
        f3.push_back(b3_id);
        for (long i = 0; i < nx; ++i) f3.push_back(nx * ny - 1 - i);

        std::vector<long> f4;
        f4.push_back(b1_id);
        f4.push_back(b4_id);
        for (long i = ny - 1; i >= 0; --i) f4.push_back(nx * i);

        std::vector<long> f5;
        f5.push_back(b1_id);
        f5.push_back(b2_id);
        f5.push_back(b3_id);
        f5.push_back(b4_id);

        faces.emplace_back(rs::actions::reverse(f1));
        faces.emplace_back(rs::actions::reverse(f2));
        faces.emplace_back(rs::actions::reverse(f3));
        faces.emplace_back(rs::actions::reverse(f4));
        faces.emplace_back(rs::actions::reverse(f5));
    }
    logger->trace("Mesh data generated!");

    return {vertices, faces};
}

}  // namespace

void
mdv::mesh::create_closed_from_function(
        const std::filesystem::path&          destination,
        std::function<double(double, double)> f,
        const double                          thickness,
        const DrawableFunctionParameters&     parameters
) {
    Expects(thickness > 0.0);
    const auto [vertices, faces] = build_surface(f, parameters, thickness);
    write_off(destination, vertices, faces);
}

std::filesystem::path
mdv::mesh::create_closed_from_function(
        std::function<double(double, double)> f,
        const double                          thickness,
        const DrawableFunctionParameters&     parameters
) {
    static int mesh_id = 0;

    const std::string mesh_name = fmt::format("closed_mesh_{}.off", mesh_id);
    ++mesh_id;
    const std::filesystem::path dest =
            std::filesystem::temp_directory_path() / mesh_name;

    const auto [vertices, faces] = build_surface(f, parameters, thickness);
    write_off(dest, vertices, faces);
    return dest;
}

void
mdv::mesh::create_from_function(
        const std::filesystem::path&          destination,
        std::function<double(double, double)> f,
        const DrawableFunctionParameters&     parameters
) {
    const auto [vertices, faces] = build_surface(f, parameters);
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
    for (std::size_t i = 0; i < n_edges - 1; ++i)
        faces.emplace_back(std::vector<long>{0, i + 1, i + 2});
    faces.emplace_back(std::vector<long>{0, n_edges, 1});

    write_off(res, vertices, faces);
    return res;
}

std::filesystem::path
mdv::mesh::create_flat(const double size) {
    const std::filesystem::path res =
            std::filesystem::temp_directory_path() / "flat.off";

    DrawableFunctionParameters params{
            .x_range                = std::make_pair(-size / 2, size / 2),
            .y_range                = std::make_pair(-size / 2, size / 2),
            .x_discretisation_steps = 2,
            .y_discretisation_steps = 2,
    };
    const auto zero_func = [](const double x, const double y) -> double { return 0.0; };
    return create_from_function(zero_func, params);
}
