#include <algorithm>
#include <chrono>
#include <random>
#include <string>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/sampler.hpp"

using namespace mdv::mesh;

int
main(int argc, char* argv[]) {
    using namespace std::chrono;

    std::string mesh_name = "bunny_simple.off";
    if (argc > 1) mesh_name = std::string(argv[1]);  // NOLINT pointer arithmethic

    const auto mesh    = Mesh::from_file(mdv::config::meshes_directory() / mesh_name);
    auto       sampler = Sampler(mesh, 0);

    std::size_t num_points = 500;
    const auto  pt_a       = sampler.points_in_faces(num_points);
    const auto  pt_b       = sampler.points_in_faces(num_points);

    std::vector<double> dg(num_points, 0.0);
    std::vector<double> e_le(num_points, 0.0);

    auto start = high_resolution_clock::now();
    for (std::size_t i = 0; i < num_points; ++i) {
        const auto&  p1      = pt_a[i];
        const auto&  p2      = pt_b[i];
        const auto   log     = logarithmic_map(mesh, p1, p2);
        const auto   exp     = exponential_map(mesh, log);
        const double d_p1_p2 = log.cartesian_vector().norm();
        const double d_p2_pexp =
                logarithmic_map(mesh, p2, exp).cartesian_vector().norm();

        dg[i]   = d_p1_p2;
        e_le[i] = d_p2_pexp / d_p1_p2;
        if (i % 20 == 0) fmt::print("{}\n", i);
    }
    auto stop = high_resolution_clock::now();
    fmt::print("e_le computed in {}s\n", duration_cast<seconds>(stop - start).count());

    const double dgmax = std::ranges::max(dg);
    fmt::print("Max geodesic distance: {}\n", dgmax);

    std::vector<TangentVector> vectors;
    vectors.reserve(num_points);
    std::mt19937_64                        generator{0};  // NOLINT
    std::uniform_real_distribution<double> scaling(1e-3, dgmax * 0.5);
    for (const auto& p : pt_a) {
        vectors.emplace_back(sampler.unit_tangent(p));
        vectors.back().scale(scaling(generator));
    }
    std::vector<double> vec_norms(num_points, 0.0);
    std::vector<double> e_el(num_points, 0.0);

    start = high_resolution_clock::now();
    for (std::size_t i = 0; i < num_points; ++i) {
        const auto&  p     = pt_a[i];
        const auto&  v     = vectors[i];
        const double vnorm = v.cartesian_vector().norm();
        const auto   log   = logarithmic_map(mesh, exponential_map(mesh, v), p);

        vec_norms[i] = vnorm;
        e_le[i]      = (log.cartesian_vector() - v.cartesian_vector()).norm() / vnorm;
        if (i % 20 == 0) fmt::print("{}\n", i);
    }
    stop = high_resolution_clock::now();
    fmt::print("e_el computed in {}s\n", duration_cast<seconds>(stop - start).count());


    return 0;
}
