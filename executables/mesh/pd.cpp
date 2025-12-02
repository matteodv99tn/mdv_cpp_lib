#include <fmt/os.h>
#include <iostream>
#include <string>

#include "mdv/config.hpp"
#include "mdv/mesh/kernel.hpp"
#include "mdv/mesh/mesh.hpp"

using namespace mdv::mesh;

using std::filesystem::path;

int
main(int argc, char* argv[]) {
    std::string mesh_path = mdv::config::meshes_directory() / "angle.off";
    const auto  mesh      = Mesh::from_file(mesh_path);

    std::vector<std::pair<std::size_t, std::size_t>> tests{
            {10, 500   },
            {10, 100000},
            // {10,   2000},
            // {20,   500 },
            // {100,  3000 },
            // {200,  10000 },
            // {400,  500 },
    };

    MeshKernel kernel(mesh);

    for (const auto [n1, n2] : tests) {
        std::cout << n1 << " x " << n2 << " -> "
                  << kernel.find_max_lengthscale(1.0, 10.0, n1, n2, false) << "\n";
    }

    return 0;
}
