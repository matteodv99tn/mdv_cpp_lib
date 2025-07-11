#include "mdv/mesh/mesh_utilities.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <fstream>
#include <vector>

#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/tangent_vector.hpp"

using FaceData   = std::vector<Eigen::Vector3i>;
using VertexData = std::vector<Eigen::Vector3d>;

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

std::string
mdv::mesh::create_saddle() {
    const std::filesystem::path res = std::filesystem::path("/tmp") / "saddle.off";

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

std::string
mdv::mesh::create_cone(const std::size_t n_edges, const double axis_from_angle_deg) {
    const std::string           filename = fmt::format("cone_{}.off", n_edges);
    const std::filesystem::path res      = std::filesystem::path("/tmp") / filename;

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
