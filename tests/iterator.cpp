#include <chrono>
#include <gsl/assert>
#include <iostream>
#include <string>

#include <mdv/mesh/mesh.hpp>

int
main() {
    const std::string file_path = "/home/matteo/Documents/cpp/mdv/meshes/bunny.off";
    // const std::string   file_path =
    // "/home/matteo/Documents/cpp/mdv/meshes/bunny_simple.off";
    std::vector<double> times_ms;
    auto                start = std::chrono::high_resolution_clock::now();
    std::cout << "Loading mesh from file " << file_path << std::endl;
    mdv::mesh::Mesh     m(file_path);
    auto                stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    double delta  = static_cast<double>(duration.count()) / 1e6;

    std::cout << delta << "ms" << std::endl;
    return 0;
}
