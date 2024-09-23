#include <chrono>
#include <gsl/assert>
#include <iostream>
#include <string>

#include <mdv/mesh/mesh.hpp>

int
main() {
    // const std::string file_path =
    //         "/home/matteo/Documents/cpp/mdv/meshes/generated/flat.off";
    const std::string file_path = "/home/matteo/Documents/cpp/mdv/meshes/bunny.off";
    // const std::string   file_path =
    // "/home/matteo/Documents/cpp/mdv/meshes/bunny_simple.off";
    std::vector<double> times_ms;
    auto                start = std::chrono::high_resolution_clock::now();
    mdv::mesh::Mesh     m(file_path);
    auto                stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    double delta  = static_cast<double>(duration.count()) / 1e6;

    std::cout << delta << "ms" << std::endl;

    Eigen::Vector3d pt{-0.16234, 0.58043, 0.00105834};
    auto            is_approx = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return (a - b).norm() < 1e-5;
    };

    start             = std::chrono::high_resolution_clock::now();
    const auto vertex = std::find_if(
            m.vertices_begin(),
            m.vertices_end(),
            [&pt, &is_approx](const auto& v) { return is_approx(v.position(), pt); }
    );
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    delta    = static_cast<double>(duration.count()) / 1e6;
    std::cout << "Iterator search: " << delta << "ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (long i = 0; i < m.num_vertices(); ++i) {
        if (is_approx(m.vertex(i).position(), pt)) { break; }
    }
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    delta    = static_cast<double>(duration.count()) / 1e6;
    std::cout << "Semi-manual search: " << delta << "ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (long i = 0; i < m.num_vertices(); ++i) {
        if (is_approx(m._v_mat.col(i), pt)) { break; }
    }
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    delta    = static_cast<double>(duration.count()) / 1e6;
    std::cout << "Manual search: " << delta << "ms" << std::endl;


    return 0;
}
