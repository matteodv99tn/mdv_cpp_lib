#ifndef MDV_CONFIG_HPP
#define MDV_CONFIG_HPP

#include <filesystem>

namespace mdv::config {

std::filesystem::path meshes_directory();

std::filesystem::path letter_dataset_directory();


}  // namespace mdv::config


#endif  // MDV_CONFIG_HPP
