#include "mdv/mesh/kernel.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdexcept>
#include <thread>
#include <utility>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"

namespace mdv::mesh {

class DistanceEvaluator {
public:
    using InputVector             = MeshKernel::InputVector;
    using ShortestPath            = internal::CgalImpl::ShortestPath;
    using CgalGeodesicConstructor = internal::CgalGeodesicConstructor;

    Eigen::MatrixXd
    operator()(const InputVector& pts1, const InputVector& pts2) const {
        Eigen::MatrixXd   res(pts1.size(), pts2.size());
        const std::size_t n_threads = 30;
        const long batch_size       = std::max(std::size_t{2}, pts1.size() / n_threads);

        auto thread_evaluator = [this, &res, &pts1, &pts2, &batch_size](const long i) {
            evaluate_rows_batched(res, pts1, pts2, i, batch_size);
        };
        std::vector<std::thread> threads;
        threads.reserve(n_threads + 1);

        for (long i = 0; i < res.rows(); i += batch_size)
            threads.emplace_back(thread_evaluator, i);
        for (auto& th : threads) th.join();
        return res;
    }

private:
    static void
    evaluate_rows_batched(
            Eigen::MatrixXd&   res,
            const InputVector& pts1,
            const InputVector& pts2,
            const long         row_start_id,
            const long         n_rows
    ) {
        for (long i = row_start_id; i < std::min(row_start_id + n_rows, res.rows());
             ++i) {
            ShortestPath shpath(internal::get_mesh_impl(pts1.front().mesh()));
            CgalGeodesicConstructor::set_source(shpath, pts1[i]);

            for (long j = 0; j < res.cols(); ++j) {
                const auto geod =
                        CgalGeodesicConstructor::construct_geodesic(shpath, pts2[j]);
                res(i, j) = length(geod);
            }
        }
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
            for (const auto& pt : pts1) {
                shpath_objs.emplace_back(m);
                CgalGeodesicConstructor::set_source(shpath_objs.back(), pt);
            }
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

        std::vector<std::thread> threads;
        threads.reserve(res.rows());

        auto row_processor = [&data, &res, &pts2](const long i) {
            for (long j = 0; j < res.cols(); ++j) {
                const auto path = CgalGeodesicConstructor::construct_geodesic(
                        data.shpath_objs[i], pts2[j]
                );
                res(i, j) = length(path);
            }
        };

        for (long i = 0; i < res.rows(); ++i) threads.emplace_back(row_processor, i);
        for (auto& th : threads) th.join();
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
