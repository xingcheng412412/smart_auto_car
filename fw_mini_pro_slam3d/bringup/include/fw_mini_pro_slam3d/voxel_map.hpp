#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace fw_mini_pro_slam3d {
struct MapPoint { float x, y, z, intensity; };
static_assert(sizeof(MapPoint) == 16, "PCD requires packed floats");
class VoxelMap {
  using Key = std::array<int64_t, 3>;
  struct Hash {
    size_t operator()(const Key& k) const {
      size_t h = 0;
      for (auto v : k) h ^= std::hash<int64_t>{}(v) + 0x9e3779b9 + (h<<6) + (h>>2);
      return h;
    }
  };
  struct Cell { double x=0, y=0, z=0, intensity=0; uint64_t count=0; };
 public:
  VoxelMap(double resolution, size_t limit) : resolution_(resolution), limit_(limit) {
    if (!std::isfinite(resolution) || resolution <= 0 || !limit)
      throw std::invalid_argument("voxel_size and max_voxels must be positive");
  }
  void add(const MapPoint& p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.intensity)) return;
    Key key;
    const std::array<double, 3> xyz{p.x, p.y, p.z};
    for (size_t i=0;i<3;++i) {
      const double value=std::floor(xyz[i]/resolution_);
      // Conservative safe range for double -> int64 conversion.
      if (!std::isfinite(value) || std::abs(value) >= 9e18) return;
      key[i]=static_cast<int64_t>(value);
    }
    auto it=cells_.find(key);
    if (it==cells_.end()) {
      if (cells_.size()>=limit_) { truncated_=true; return; }
      it=cells_.emplace(key,Cell{}).first;
    }
    auto& c=it->second;
    c.x+=p.x; c.y+=p.y; c.z+=p.z; c.intensity+=p.intensity; ++c.count;
  }
  size_t size() const { return cells_.size(); }
  bool truncated() const { return truncated_; }
  std::vector<MapPoint> points() const {
    std::vector<MapPoint> out; out.reserve(size());
    for (const auto& item:cells_) {
      const auto& c=item.second; const double n=static_cast<double>(c.count);
      out.push_back({float(c.x/n),float(c.y/n),float(c.z/n),float(c.intensity/n)});
    }
    return out;
  }
  void save(const std::string& filename) const {
    if (cells_.empty()) throw std::runtime_error("No registered points received; map is empty");
    const std::filesystem::path dest(filename);
    if (!dest.is_absolute() || dest.filename().empty() || std::filesystem::is_directory(dest))
      throw std::invalid_argument("map_file_path must be an absolute file path");
    if (!dest.parent_path().empty()) std::filesystem::create_directories(dest.parent_path());
    auto tmp=dest; tmp += ".tmp";
    try {
      std::ofstream out(tmp,std::ios::binary|std::ios::trunc);
      out.exceptions(std::ios::failbit|std::ios::badbit);
      out << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH "
          << size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << size() << "\nDATA binary\n";
      for (const auto& p:points()) out.write(reinterpret_cast<const char*>(&p),sizeof(p));
      out.close();
      std::filesystem::rename(tmp,dest);
    } catch (...) {
      std::error_code ec; std::filesystem::remove(tmp,ec); throw;
    }
  }
 private:
  double resolution_; size_t limit_; bool truncated_=false;
  std::unordered_map<Key,Cell,Hash> cells_;
};
}
