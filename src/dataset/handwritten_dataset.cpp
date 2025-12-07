#include "mdv/dataset/handwritten_dataset.hpp"

#define MATIOCPP_HAS_EIGEN

#include <cctype>
#include <filesystem>
#include <fmt/format.h>
#include <map>
#include <matioCpp/matioCpp.h>

#include "mdv/config.hpp"

namespace mdv {

namespace {
    using path = std::filesystem::path;

    std::pair<long, long>
    find_start_end_indices(const Eigen::MatrixXd& vel_data, const double v_th) {
        const auto exceeds_threshold = [&v_th](const auto& v) -> bool {
            return v.norm() > v_th;
        };

        auto start_iter = std::find_if(
                vel_data.colwise().begin(), vel_data.colwise().end(), exceeds_threshold
        );
        auto end_iter = std::find_if(
                vel_data.colwise().rbegin(),
                vel_data.colwise().rend(),
                exceeds_threshold
        );

        const long start_id = std::distance(vel_data.colwise().begin(), start_iter);
        const long end_id   = vel_data.cols()
                            - std::distance(vel_data.colwise().rbegin(), end_iter) - 1;

        return {start_id, end_id};
    }

    Eigen::MatrixXd
    get_columns_inbetween(
            const Eigen::MatrixXd& in, const std::pair<long, long> range
    ) {
        const auto& [start_id, end_id] = range;
        return in.middleCols(start_id, end_id - start_id + 1);
    }

    LetterDemonstration
    process_demonstration(
            const matioCpp::Struct& mat_struct,
            const double            vel_threshold,
            const double            scale
    ) {
        auto get_as_matrix =
                [&mat_struct](const std::string& label) -> Eigen::MatrixXd {
            return matioCpp::to_eigen(
                    mat_struct[label].asMultiDimensionalArray<double>()
            );
        };

        const Eigen::MatrixXd pos = scale * get_as_matrix("pos");
        const Eigen::MatrixXd vel = scale * get_as_matrix("vel");
        const Eigen::MatrixXd acc = scale * get_as_matrix("acc");

        const auto range = find_start_end_indices(vel, vel_threshold);

        LetterDemonstration res;
        res.pos = get_columns_inbetween(pos, range);
        res.vel = get_columns_inbetween(vel, range);
        res.acc = get_columns_inbetween(acc, range);
        return res;
    }

    LetterDemoCollections
    process_letter_dataset(
            const path& file, const double vel_threshold, const double scale
    ) {
        matioCpp::File      mat_file(file);
        matioCpp::CellArray demos   = mat_file.read("demos").asCellArray();
        const std::size_t   n_demos = demos.numberOfElements();

        LetterDemoCollections res;
        res.demonstrations.reserve(n_demos);
        for (std::size_t i = 0; i < n_demos; ++i) {
            res.demonstrations.emplace_back(
                    process_demonstration(demos(i).asStruct(), vel_threshold, scale)
            );
        }

        return res;
    }

}  // namespace

std::map<char, LetterDemoCollections>
load_letter_dataset(const double vel_threshold, const double scale) {
    const auto base_path = config::letter_dataset_directory();

    std::map<char, LetterDemoCollections> res;
    for (char letter = 'A'; letter <= 'Z'; ++letter) {
        const path file_path = base_path / path(fmt::format("{}.mat", letter));
        res.emplace(
                std::tolower(letter),
                process_letter_dataset(file_path, vel_threshold, scale)
        );
    }
    return res;
}

}  // namespace mdv
