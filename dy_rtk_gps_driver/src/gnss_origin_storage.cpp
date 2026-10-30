#include "rtk_gps_driver_cpp/gnss_origin_storage.h"

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace rtk_gps_driver_cpp {
namespace {

void set_error(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool origin_key(const std::string& line, std::string* key) {
  if (line.empty() || line.front() == '#' || line.front() == ' ' ||
      line.front() == '\t') {
    return false;
  }

  const std::size_t colon = line.find(':');
  if (colon == std::string::npos) {
    return false;
  }

  std::size_t key_end = colon;
  while (key_end > 0 &&
         (line[key_end - 1] == ' ' || line[key_end - 1] == '\t')) {
    --key_end;
  }
  const std::string candidate = line.substr(0, key_end);
  if (candidate != "lat" && candidate != "lon" && candidate != "h") {
    return false;
  }
  *key = candidate;
  return true;
}

std::string formatted_origin_line(const std::string& original,
                                  const std::string& key,
                                  double value) {
  std::ostringstream formatted;
  formatted << key << ": " << std::fixed
            << std::setprecision(key == "h" ? 6 : 12) << value;

  const std::size_t colon = original.find(':');
  const std::size_t comment = original.find('#', colon + 1);
  if (comment != std::string::npos) {
    std::size_t suffix_start = comment;
    while (suffix_start > colon + 1 &&
           (original[suffix_start - 1] == ' ' ||
            original[suffix_start - 1] == '\t')) {
      --suffix_start;
    }
    formatted << original.substr(suffix_start);
  }
  return formatted.str();
}

bool write_all(int file_descriptor, const std::string& contents,
               std::string* error) {
  std::size_t offset = 0;
  while (offset < contents.size()) {
    const ssize_t written =
        ::write(file_descriptor, contents.data() + offset,
                contents.size() - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    set_error("Failed to write temporary GNSS origin file: " +
                  std::string(std::strerror(errno)),
              error);
    return false;
  }
  return true;
}

}  // namespace

bool update_gnss_origin_yaml(const std::string& config_path,
                             double latitude,
                             double longitude,
                             double altitude,
                             std::string* error) {
  if (error != nullptr) {
    error->clear();
  }
  if (config_path.empty()) {
    set_error("gnss_config_file is empty", error);
    return false;
  }
  if (!std::isfinite(latitude) || latitude < -90.0 || latitude > 90.0 ||
      !std::isfinite(longitude) || longitude < -180.0 ||
      longitude > 180.0 || !std::isfinite(altitude)) {
    set_error("GNSS origin coordinates are invalid", error);
    return false;
  }

  char resolved_buffer[PATH_MAX];
  if (::realpath(config_path.c_str(), resolved_buffer) == nullptr) {
    set_error("Failed to resolve GNSS origin file '" + config_path + "': " +
                  std::string(std::strerror(errno)),
              error);
    return false;
  }
  const std::string resolved_path(resolved_buffer);

  struct stat original_stat;
  if (::stat(resolved_path.c_str(), &original_stat) != 0 ||
      !S_ISREG(original_stat.st_mode)) {
    set_error("GNSS origin path is not a regular file: " + resolved_path,
              error);
    return false;
  }

  std::ifstream input(resolved_path);
  if (!input.is_open()) {
    set_error("Failed to open GNSS origin file: " + resolved_path, error);
    return false;
  }

  bool seen_latitude = false;
  bool seen_longitude = false;
  bool seen_altitude = false;
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::string key;
    if (origin_key(line, &key)) {
      bool* seen = key == "lat" ? &seen_latitude
                    : key == "lon" ? &seen_longitude
                                     : &seen_altitude;
      if (*seen) {
        set_error("Duplicate root YAML key: " + key, error);
        return false;
      }
      *seen = true;
      const double value = key == "lat" ? latitude
                           : key == "lon" ? longitude
                                          : altitude;
      line = formatted_origin_line(line, key, value);
    }
    lines.push_back(line);
  }
  if (input.bad()) {
    set_error("Failed while reading GNSS origin file: " + resolved_path,
              error);
    return false;
  }

  if (!seen_latitude) {
    lines.push_back(formatted_origin_line("lat:", "lat", latitude));
  }
  if (!seen_longitude) {
    lines.push_back(formatted_origin_line("lon:", "lon", longitude));
  }
  if (!seen_altitude) {
    lines.push_back(formatted_origin_line("h:", "h", altitude));
  }

  std::ostringstream rendered;
  for (const std::string& rendered_line : lines) {
    rendered << rendered_line << '\n';
  }

  std::string temporary_template = resolved_path + ".tmp.XXXXXX";
  std::vector<char> temporary_path(temporary_template.begin(),
                                   temporary_template.end());
  temporary_path.push_back('\0');
  const int file_descriptor = ::mkstemp(temporary_path.data());
  if (file_descriptor < 0) {
    set_error("Failed to create temporary GNSS origin file: " +
                  std::string(std::strerror(errno)),
              error);
    return false;
  }

  bool success = true;
  if (::fchmod(file_descriptor, original_stat.st_mode & 07777) != 0) {
    set_error("Failed to preserve GNSS origin file permissions: " +
                  std::string(std::strerror(errno)),
              error);
    success = false;
  }
  const std::string contents = rendered.str();
  if (success && !write_all(file_descriptor, contents, error)) {
    success = false;
  }
  if (success && ::fsync(file_descriptor) != 0) {
    set_error("Failed to flush temporary GNSS origin file: " +
                  std::string(std::strerror(errno)),
              error);
    success = false;
  }
  if (::close(file_descriptor) != 0 && success) {
    set_error("Failed to close temporary GNSS origin file: " +
                  std::string(std::strerror(errno)),
              error);
    success = false;
  }

  if (!success) {
    ::unlink(temporary_path.data());
    return false;
  }
  if (::rename(temporary_path.data(), resolved_path.c_str()) != 0) {
    set_error("Failed to replace GNSS origin file: " +
                  std::string(std::strerror(errno)),
              error);
    ::unlink(temporary_path.data());
    return false;
  }

  const std::size_t slash = resolved_path.find_last_of('/');
  const std::string directory =
      slash == 0 ? "/" : resolved_path.substr(0, slash);
#ifdef O_DIRECTORY
  const int directory_descriptor =
      ::open(directory.c_str(), O_RDONLY | O_DIRECTORY);
#else
  const int directory_descriptor = ::open(directory.c_str(), O_RDONLY);
#endif
  if (directory_descriptor >= 0) {
    ::fsync(directory_descriptor);
    ::close(directory_descriptor);
  }
  return true;
}

}  // namespace rtk_gps_driver_cpp
