#include "mdv/mesh/kernel.hpp"

#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdexcept>
#include <utility>

#include <range/v3/all.hpp>

#include "BS_thread_pool.hpp"
#include "mdv/macros.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"

namespace rs = ::ranges;

namespace mdv::mesh {

struct TimeBenchmarker {
    using Time            = decltype(std::chrono::high_resolution_clock::now());
    using TimeDurationVec = std::vector<std::size_t>;

    static Time
    now() {
        return std::chrono::high_resolution_clock::now();
    };

    struct TimeSample {
        ~TimeSample() {
            const Time end_time = now();
            const auto delta    = end_time - start_time;
            dest_vector->emplace_back(delta.count());
        }

        Time             start_time;
        TimeDurationVec* dest_vector;
    };

    MDV_NODISCARD TimeSample
    operator()(const long rows, const long cols) {
        TimeDurationVec& vec = times[std::make_pair(rows, cols)];
        return TimeSample{.start_time = now(), .dest_vector = &vec};
    }

    std::map<std::pair<long, long>, TimeDurationVec> times;

    static std::size_t
    mean(const TimeDurationVec& ts) {
        return rs::accumulate(ts, std::size_t{0}) / ts.size();
    }

    ~TimeBenchmarker() {
        // Display mean / median / other data when destroying
        // fmt::println("Showing logged data");
        // for(const auto& [dims, ts]: times){
        //     const auto& [rows, cols] = dims;
        //     const double time_ms = double(mean(ts)) * 1e-6;
        //     fmt::println("{} x {} -> {:.4}ms", rows, cols, time_ms);
        // }
        // fmt::println("Showing logged data --- Done");
    }
};

static TimeBenchmarker benchmarker;

namespace {
    BS::thread_pool th_pool;
}  // namespace

class DistanceEvaluator {
public:
    using InputVector             = MeshKernel::InputVector;
    using ShortestPath            = internal::CgalImpl::ShortestPath;
    using CgalGeodesicConstructor = internal::CgalGeodesicConstructor;

    Eigen::MatrixXd
    operator()(const InputVector& pts1, const InputVector& pts2) const {
        Eigen::MatrixXd res(pts1.size(), pts2.size());
        th_pool.detach_loop(0, res.rows(), [&](const long i) {
            evaluate_row(i, res, pts1, pts2);
        });
        th_pool.wait();
        return res;
    }

private:
    static void
    evaluate_row(
            const long         i,
            Eigen::MatrixXd&   res,
            const InputVector& pts1,
            const InputVector& pts2
    ) {
        ShortestPath shpath(internal::get_mesh_impl(pts1.front().mesh()));
        CgalGeodesicConstructor::set_source(shpath, pts1[i]);

        for (long j = 0; j < res.cols(); ++j) {
            const auto geod =
                    CgalGeodesicConstructor::construct_geodesic(shpath, pts2[j]);
            res(i, j) = length(geod);
        }
    }

    static void
    evaluate_rows_batched(
            Eigen::MatrixXd&   res,
            const InputVector& pts1,
            const InputVector& pts2,
            const long         row_start_id,
            const long         n_rows
    ) {
        const long& start = row_start_id;
        const long  end   = std::min(row_start_id + n_rows, res.rows());
        for (long i = start; i < end; ++i) evaluate_row(i, res, pts1, pts2);
    }
};

namespace internal {
    struct Data {
        using InputVector  = MeshKernel::InputVector;
        using ShortestPath = internal::CgalImpl::ShortestPath;

        InputVector               pts1;
        std::vector<ShortestPath> shpath_objs;

        Data(InputVector points) : pts1(std::move(points)) {
            using internal::CgalGeodesicConstructor;

            const auto& mesh = pts1[0].mesh();
            const auto& m    = internal::get_mesh_impl(pts1[0].mesh());

            shpath_objs.reserve(pts1.size());
            for (const auto& pt : pts1) shpath_objs.emplace_back(m);

            const auto process_i = [&](const long i) {
                CgalGeodesicConstructor::set_source(shpath_objs[i], pts1[i]);
                CgalGeodesicConstructor::construct_geodesic(
                        shpath_objs[i], mesh.vertex(0)
                );
            };
            th_pool.detach_loop(0, pts1.size(), process_i);
            th_pool.wait();
        }
    };

    Eigen::MatrixXd
    eval_sek(const Eigen::MatrixXd& distance_matrix, const double ls) {
        const double    lambda = 0.5 / (ls * ls);
        Eigen::MatrixXd res    = distance_matrix.unaryExpr([lambda](const double x) {
            return std::exp(-lambda * x * x);
        });
        return res;
    }

    void
    eval_sek_inplace(Eigen::MatrixXd& distance_matrix, const double ls) {
        const double lambda = 0.5 / (ls * ls);
        distance_matrix     = distance_matrix.unaryExpr([lambda](const double x) {
            return std::exp(-lambda * x * x);
        });
    }

    bool
    is_positive_definite(const Eigen::MatrixXd& mat) {
        // Check if it is possible to perform a valid Cholesky decomposition
        Eigen::LLT<Eigen::MatrixXd> llt(mat);
        return llt.info() == Eigen::Success;
    }


}  // namespace internal

namespace {

    using ShortestPath = internal::CgalImpl::ShortestPath;
    using internal::CgalGeodesicConstructor;

    void
    process_distance_matrix(
            internal::Data&                    data,
            const internal::Data::InputVector& pts2,
            Eigen::MatrixXd&                   res
    ) {
        using internal::CgalGeodesicConstructor;
        auto time_eval = benchmarker(data.pts1.size(), pts2.size());

        auto row_processor = [&data, &res, &pts2](const long i) {
            for (long j = 0; j < res.cols(); ++j) {
                const auto path = CgalGeodesicConstructor::construct_geodesic(
                        data.shpath_objs[i], pts2[j]
                );
                res(i, j) = length(path);
            }
        };

        th_pool.detach_loop(0, res.rows(), row_processor);
        th_pool.wait();
    }
}  // namespace

MeshKernel::MeshKernel(const Mesh& mesh) : _mesh(&mesh) {
}

Eigen::MatrixXd
MeshKernel::distance_matrix(const InputVector& pts1, const InputVector& pts2) const {
    const Eigen::MatrixXd D12 = DistanceEvaluator{}(pts1, pts2);
    const Eigen::MatrixXd D21 = DistanceEvaluator{}(pts2, pts1);
    assert(D12.cols() == D21.rows() && D12.rows() == D21.cols());

    const long      nr = D12.rows();
    const long      nc = D12.cols();
    Eigen::MatrixXd res(nr, nc);
    for (long i = 0; i < nr; ++i) {
        for (long j = 0; j < nc; ++j) { res(i, j) = std::min(D12(i, j), D21(j, i)); }
    }
    return res;
}

double
MeshKernel::find_max_lengthscale(
        const double      ls0,
        const double      lsmax,
        const std::size_t num_points,
        const std::size_t num_tests,
        const bool        verbose
) {
    assert(_mesh);
    if (verbose) {
        std::cout << "Finding maximum lengthscale for mesh " << _mesh->name() << "\n";
        std::cout << "Number of random testing conditions: " << num_tests << "\n";
        std::cout << "Number of points per testing condition: " << num_points << "\n";
    }

    std::vector<Eigen::MatrixXd> dist_matrices;
    dist_matrices.reserve(num_tests);
    for (std::size_t i = 0; i < num_tests; ++i) {
        std::vector<mdv::mesh::Point> points;
        points.reserve(num_points);
        for (std::size_t j = 0; j < num_points; ++j)
            points.emplace_back(mdv::mesh::Point::random(*_mesh));

        dist_matrices.emplace_back(distance_matrix(points, points));
    }

    double ls_min = 0.0;
    double ls_max = lsmax;
    double ls     = ls0;

    if (verbose) std::cout << "Starting search\n";
    for (std::size_t k = 0; k < 50; ++k) {
        ls = 0.5 * (ls_min + ls_max);

        bool is_pd = true;
        for (std::size_t i = 0; i < dist_matrices.size(); ++i) {
            const Eigen::MatrixXd ker = internal::eval_sek(dist_matrices[i], ls);
            if (!internal::is_positive_definite(ker)) {
                is_pd = false;
                break;
            }
        }

        if (is_pd) ls_min = ls;
        else ls_max = ls;
    }

    return ls_min;
}

double
MeshKernel::find_pointset_max_lengthscale(
        const InputVector& pts, const std::size_t num_steps
) const {
    assert(_mesh);

    const Eigen::MatrixXd dist_matrix = distance_matrix(pts, pts);
    return find_matrix_max_lengthscale(dist_matrix, num_steps, 0.1);
}

InexactMeshKernel::~InexactMeshKernel() {
    delete _data;
};

Eigen::MatrixXd
InexactMeshKernel::distance_matrix(
        const InputVector& pts1, const InputVector& pts2
) const {
    auto time_eval = benchmarker(pts1.size(), pts2.size());
    if (pts1.size() < pts2.size()) return DistanceEvaluator{}(pts1, pts2);
    return DistanceEvaluator{}(pts2, pts1).transpose();
}

Eigen::MatrixXd
InexactMeshKernel::distance_matrix(const InputVector& pts2) {
    if (_data == nullptr)
        throw std::runtime_error("InexactMeshKernel: cache not properly initialised!");

    assert(_data->shpath_objs.size() == _data->pts1.size());
    Eigen::MatrixXd distances(_data->pts1.size(), pts2.size());
    process_distance_matrix(*_data, pts2, distances);
    return distances;
}

Eigen::MatrixXd
InexactMeshKernel::evaluate_single(const InputVector& pts2, const double ls) {
    if (_data == nullptr)
        throw std::runtime_error("InexactMeshKernel: cache not properly initialised!");

    Eigen::MatrixXd distances = distance_matrix(pts2);
    internal::eval_sek_inplace(distances, ls);
    return distances;
}

void
InexactMeshKernel::set_points1(const InputVector& pts1) {
    delete _data;
    _data = new Data(pts1);
}

double
find_matrix_max_lengthscale(
        const Eigen::MatrixXd& dist_matrix,
        const std::size_t      num_steps,
        const double           ls0
) {
    Eigen::MatrixXd kernel;

    auto is_pd_kernel = [&dist_matrix, &kernel](const double ls) -> bool {
        kernel = internal::eval_sek(dist_matrix, ls);
        return internal::is_positive_definite(kernel);
    };

    double ls_min = -1.0;
    double ls_max = ls0;

    while (is_pd_kernel(ls_max)) {
        ls_min = ls_max;
        ls_max *= 2.0;
    }

    std::size_t iter_count = 0;
    while (ls_min < 0.0) {
        if (is_pd_kernel(0.5 * ls_max)) {
            ls_min = 0.5 * ls_max;
            ls_max *= 2.0;  // to compensate the subsequent halving of ls_max
        }
        ls_max *= 0.5;
        ++iter_count;

        if (iter_count >= num_steps) {
            const auto msg = fmt::format(
                    "Unable to find good initial condition for the algorithm in {} "
                    "iterations",
                    num_steps
            );
            throw std::runtime_error(msg.c_str());
        }
    }


    for (std::size_t k = 0; k < num_steps; ++k) {
        const double ls = 0.5 * (ls_min + ls_max);
        if (is_pd_kernel(ls)) ls_min = ls;
        else ls_max = ls;
    }

    return ls_min;
}


}  // namespace mdv::mesh
