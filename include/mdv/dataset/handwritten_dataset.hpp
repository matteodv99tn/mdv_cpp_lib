#ifndef MDV_HANDWRITTEN_DATASET_SUPPORT_HPP
#define MDV_HANDWRITTEN_DATASET_SUPPORT_HPP

#include <Eigen/Dense>
#include <map>

namespace mdv {

struct LetterDemonstration {
    Eigen::MatrixXd pos;  // 2xN matrix
    Eigen::MatrixXd vel;  // 2xN matrix
    Eigen::MatrixXd acc;  // 2xN matrix
};

struct LetterDemoCollections {
    std::vector<LetterDemonstration> demonstrations;
};

std::map<char, LetterDemoCollections> load_letter_dataset(
        double vel_threshold = 0.01, double scale = 0.1
);

}  // namespace mdv


#endif  // MDV_HANDWRITTEN_DATASET_SUPPORT_HPP
