#ifndef MDV_FILESYSTEM_HPP
#define MDV_FILESYSTEM_HPP

#include <filesystem>

namespace mdv::utils {

using Path = std::filesystem::path;

std::string file_extension(const Path& file);
std::string file_name(const Path& file);
std::string file_name_we(const Path& file);
std::string parent_folder(const Path& file);

}  // namespace mdv::utils


#endif  // MDV_FILESYSTEM_HPP
