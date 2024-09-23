#ifndef MDV_FILESYSTEM_HPP
#define MDV_FILESYSTEM_HPP

#include <filesystem>

namespace mdv::utils {

using Path_t = std::filesystem::path;

std::string file_extension(const Path_t& file);
std::string file_name(const Path_t& file);
std::string file_name_we(const Path_t& file);
std::string parent_folder(const Path_t& file);

}  // namespace mdv::utils


#endif  // MDV_FILESYSTEM_HPP
