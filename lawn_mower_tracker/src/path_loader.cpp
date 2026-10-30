#include "lawn_mower_tracker/path_loader.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace lawn_mower_tracker
{
namespace
{

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n\"");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n\"");
  return value.substr(first, last - first + 1U);
}

std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> cells;
  std::stringstream stream(line);
  std::string cell;
  while (std::getline(stream, cell, ',')) {
    cells.push_back(trim(cell));
  }
  if (!line.empty() && line.back() == ',') {
    cells.emplace_back();
  }
  return cells;
}

bool parse_finite(const std::string & text, double & output)
{
  try {
    std::size_t consumed = 0U;
    output = std::stod(text, &consumed);
    return consumed == text.size() && std::isfinite(output);
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_integer(const std::string & text, std::int64_t & output)
{
  try {
    std::size_t consumed = 0U;
    output = std::stoll(text, &consumed);
    return consumed == text.size();
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace

PathLoadResult load_path_csv(const std::filesystem::path & path, const CsvColumns & columns)
{
  if (!std::isfinite(columns.stationary_velocity_threshold) ||
    columns.stationary_velocity_threshold < 0.0)
  {
    return {{}, "stationary velocity threshold is invalid"};
  }

  std::ifstream input(path);
  if (!input.is_open()) {
    return {{}, "file cannot be opened: " + path.string()};
  }

  std::string line;
  if (!std::getline(input, line)) {
    return {{}, "columns header is missing"};
  }

  std::unordered_map<std::string, std::size_t> indices;
  const auto header = split_csv(line);
  for (std::size_t index = 0U; index < header.size(); ++index) {
    indices[header[index]] = index;
  }
  if (indices.count(columns.x) == 0U || indices.count(columns.y) == 0U ||
    (columns.z_enabled && indices.count(columns.z) == 0U))
  {
    return {{}, "columns required by configuration are missing"};
  }
  const bool has_velocity = indices.count(columns.velocity) != 0U;
  const bool has_path_id = indices.count(columns.path_id) != 0U;
  const bool has_yaw = indices.count(columns.yaw) != 0U;

  PathLoadResult result;
  std::size_t row = 1U;
  while (std::getline(input, line)) {
    ++row;
    if (trim(line).empty()) {
      continue;
    }
    const auto cells = split_csv(line);
    const auto x_index = indices.at(columns.x);
    const auto y_index = indices.at(columns.y);
    const auto z_index = columns.z_enabled ? indices.at(columns.z) : 0U;
    const auto velocity_index = has_velocity ? indices.at(columns.velocity) : 0U;
    const auto path_id_index = has_path_id ? indices.at(columns.path_id) : 0U;
    const auto yaw_index = has_yaw ? indices.at(columns.yaw) : 0U;
    if (x_index >= cells.size() || y_index >= cells.size() ||
      (columns.z_enabled && z_index >= cells.size()) ||
      (has_velocity && velocity_index >= cells.size()) ||
      (has_path_id && path_id_index >= cells.size()) ||
      (has_yaw && yaw_index >= cells.size()))
    {
      return {{}, "row " + std::to_string(row) + " has too few columns"};
    }

    PathPoint point;
    if (!parse_finite(cells[x_index], point.x) || !parse_finite(cells[y_index], point.y) ||
      (columns.z_enabled && !parse_finite(cells[z_index], point.z)))
    {
      return {{}, "row " + std::to_string(row) + " contains an invalid number"};
    }
    if (has_velocity) {
      double velocity = 0.0;
      if (!parse_finite(cells[velocity_index], velocity)) {
        return {{}, "row " + std::to_string(row) + " contains an invalid " + columns.velocity};
      }
      point.direction =
        velocity > columns.stationary_velocity_threshold ? 1 :
        velocity < -columns.stationary_velocity_threshold ? -1 : 0;
    }
    if (has_path_id) {
      if (!parse_integer(cells[path_id_index], point.path_id)) {
        return {{}, "row " + std::to_string(row) + " contains an invalid " + columns.path_id};
      }
      point.has_path_id = true;
    }
    if (has_yaw) {
      if (!parse_finite(cells[yaw_index], point.yaw)) {
        return {{}, "row " + std::to_string(row) + " contains an invalid " + columns.yaw};
      }
      point.has_yaw = true;
    }
    result.points.push_back(point);
  }

  if (result.points.size() < 3U) {
    result.points.clear();
    result.error = "points: at least three valid path points are required";
  }
  return result;
}

}  // namespace lawn_mower_tracker
