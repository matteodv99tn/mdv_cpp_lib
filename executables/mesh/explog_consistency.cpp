#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fmt/format.h>
#include <limits>
#include <numeric>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <BS_thread_pool.hpp>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/sampler.hpp"
#include "mdv/mesh/tangent_vector.hpp"

using namespace mdv::mesh;

namespace {

struct Sample {
    double error;  // relative round-trip error
    double scale;  // geodesic distance (log-exp) or tangent norm (exp-log)
};

constexpr double nan = std::numeric_limits<double>::quiet_NaN();

std::string
vec_to_str(const Eigen::Vector3d& v) {
    return fmt::format("[{:.17g}, {:.17g}, {:.17g}]", v.x(), v.y(), v.z());
}

// Full-precision descriptor, enough to rebuild the point
std::string
dump(const Mesh& mesh, const Point& p) {
    const auto pos = vec_to_str(position(mesh, p));
    if (const auto* v = p.get_as<PointOnVertex>())
        return fmt::format("PointOnVertex{{v={}}} at {}", v->v.value, pos);
    if (const auto* e = p.get_as<PointOnEdge>())
        return fmt::format(
                "PointOnEdge{{he={}, t={:.17g}}} at {}", e->he.value, e->t, pos
        );
    const auto* f = p.get_as<PointInFace>();
    return fmt::format(
            "PointInFace{{f={}, bary=[{:.17g}, {:.17g}, {:.17g}]}} at {}",
            f->f.value,
            f->bary[0],
            f->bary[1],
            f->bary[2],
            pos
    );
}

std::string
dump(const Mesh& mesh, const TangentVector& v) {
    return fmt::format(
            "TangentVector{{face={}, vec={}, norm={:.17g}}} applied at {}",
            v.face().value,
            vec_to_str(v.cartesian_vector()),
            v.cartesian_vector().norm(),
            dump(mesh, v.application_point())
    );
}

// exp_p(log_p(q)) should land on q
Sample
log_exp_sample(const Mesh& mesh, const Point& p, const Point& q) {
    std::string_view             stage = "log_p(q)";
    std::optional<TangentVector> v;
    std::optional<Point>         q_hat;
    try {
        v                = logarithmic_map(mesh, p, q);
        stage            = "exp_p(v)";
        q_hat            = exponential_map(mesh, *v);
        stage            = "log_q(q_hat)";
        const double d   = v->cartesian_vector().norm();
        const double err = logarithmic_map(mesh, q, *q_hat).cartesian_vector().norm();
        return {.error = err / d, .scale = d};
    } catch (const std::runtime_error& e) {
        std::string msg = fmt::format(
                "[log-exp] {} threw: {}\n  p     = {}\n  q     = {}\n",
                stage,
                e.what(),
                dump(mesh, p),
                dump(mesh, q)
        );
        if (v) msg += fmt::format("  v     = {}\n", dump(mesh, *v));
        if (q_hat) msg += fmt::format("  q_hat = {}\n", dump(mesh, *q_hat));
        fmt::print(stderr, "{}", msg);  // single write: avoids interleaving
        return {.error = nan, .scale = nan};
    }
}

// log_p(exp_p(v)) should give back v
Sample
exp_log_sample(const Mesh& mesh, const TangentVector& v) {
    const auto&          p     = v.application_point();
    std::string_view     stage = "exp_p(v)";
    std::optional<Point> p_exp;
    try {
        p_exp              = exponential_map(mesh, v);
        stage              = "log_p(exp_p(v))";
        const auto   v_hat = logarithmic_map(mesh, p, *p_exp);
        const auto&  vec   = v.cartesian_vector();
        const double n     = vec.norm();
        return {.error = (v_hat.cartesian_vector() - vec).norm() / n, .scale = n};
    } catch (const std::runtime_error& e) {
        std::string msg = fmt::format(
                "[exp-log] {} threw: {}\n  v     = {}\n", stage, e.what(), dump(mesh, v)
        );
        if (p_exp) msg += fmt::format("  exp   = {}\n", dump(mesh, *p_exp));
        fmt::print(stderr, "{}", msg);
        return {.error = nan, .scale = nan};
    }
}

/**
 * @brief Evaluates f(mesh, i) for i in [0, n) on the thread pool.
 *
 * The mesh geodesic constructor caches state internally, so each block works on
 * its own mesh instance loaded from mesh_path.
 */
template <typename F>
std::vector<Sample>
parallel_samples(
        BS::thread_pool<>&           pool,
        std::string_view             name,
        const std::filesystem::path& mesh_path,
        std::size_t                  n,
        F&&                          f
) {
    using namespace std::chrono;

    std::vector<Sample> res(n);
    const auto          start = steady_clock::now();
    pool.submit_blocks(
                std::size_t{0},
                n,
                [&](const std::size_t first, const std::size_t last) {
                    const auto mesh = Mesh::from_file(mesh_path);
                    for (std::size_t i = first; i < last; ++i) res[i] = f(mesh, i);
                }
    ).get();
    const auto stop = steady_clock::now();
    fmt::print(
            "{} computed in {:.2f}s\n", name, duration<double>(stop - start).count()
    );
    return res;
}

std::vector<TangentVector>
random_tangents(
        Sampler&                  sampler,
        const std::vector<Point>& points,
        double                    min_norm,
        double                    max_norm,
        std::uint64_t             seed
) {
    std::mt19937_64                        generator{seed};
    std::uniform_real_distribution<double> scaling(min_norm, max_norm);

    std::vector<TangentVector> res;
    res.reserve(points.size());
    for (const auto& p : points) {
        res.emplace_back(sampler.unit_tangent(p));
        res.back().scale(scaling(generator));
    }
    return res;
}

double
quantile(const std::vector<double>& sorted, double q) {
    const auto idx =
            static_cast<std::size_t>(q * static_cast<double>(sorted.size() - 1));
    return sorted[idx];
}

void
report(std::string_view name, const std::vector<Sample>& samples) {
    std::vector<double> errors;
    errors.reserve(samples.size());
    for (const auto& s : samples)
        if (!std::isnan(s.error)) errors.push_back(s.error);
    std::ranges::sort(errors);

    fmt::print(
            "{}: {} / {} samples failed\n",
            name,
            samples.size() - errors.size(),
            samples.size()
    );
    if (errors.empty()) return;

    const double mean = std::accumulate(errors.begin(), errors.end(), 0.0)
                        / static_cast<double>(errors.size());
    fmt::print(
            "{} relative error: mean {:.3e}, median {:.3e}, p95 {:.3e}, max {:.3e}\n",
            name,
            mean,
            quantile(errors, 0.5),
            quantile(errors, 0.95),
            errors.back()
    );
}

}  // namespace

int
main(int argc, char* argv[]) {
    const std::vector<std::string> args(argv + 1, argv + argc);  // NOLINT

    const std::string       mesh_name  = args.size() > 0 ? args[0] : "bunny_simple.off";
    const std::size_t       num_points = args.size() > 1 ? std::stoul(args[1]) : 5000;
    constexpr std::uint64_t seed       = 0;

    const auto mesh_path = mdv::config::meshes_directory() / mesh_name;
    const auto mesh      = Mesh::from_file(mesh_path);
    auto       sampler   = Sampler(mesh, seed);

    const auto pt_a = sampler.points_in_faces(num_points);
    const auto pt_b = sampler.points_in_faces(num_points);

    BS::thread_pool<> pool;
    fmt::print("Using {} threads on {} points\n", pool.get_thread_count(), num_points);

    const auto le = parallel_samples(
            pool, "log-exp", mesh_path, num_points, [&](const Mesh& m, std::size_t i) {
                return log_exp_sample(m, pt_a[i], pt_b[i]);
            }
    );
    report("log-exp", le);

    double d_max = 0.0;
    for (const auto& s : le)
        if (!std::isnan(s.scale)) d_max = std::max(d_max, s.scale);
    fmt::print("Max geodesic distance: {}\n", d_max);

    const auto vectors = random_tangents(sampler, pt_a, 1e-3, 0.5 * d_max, seed);
    const auto el      = parallel_samples(
            pool, "exp-log", mesh_path, num_points, [&](const Mesh& m, std::size_t i) {
                return exp_log_sample(m, vectors[i]);
            }
    );
    report("exp-log", el);

    return 0;
}
