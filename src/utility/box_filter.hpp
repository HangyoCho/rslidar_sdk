// SDK-side box crop filter (ego-vehicle removal).
//
// Drops points whose (x, y, z) falls inside any of the configured axis-aligned
// boxes. The boxes live in the published point-cloud frame — that is, after
// the driver-side `x/y/z/roll/pitch/yaw` transform is applied — so the
// numbers you put in config/config.yaml are the same ones you read off rviz
// when you draw a marker around the vehicle outline.
//
// Filtering is intentionally simple (AABB, "remove inside any") because the
// only stated use case is masking the RC-car chassis/mast that the lidar
// otherwise sees in every frame.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "msg/rs_msg/lidar_point_cloud_msg.hpp"
#include "utility/common.hpp"

namespace robosense
{
namespace lidar
{

struct CropBox
{
  std::string name;
  float x_min{-std::numeric_limits<float>::infinity()};
  float x_max{ std::numeric_limits<float>::infinity()};
  float y_min{-std::numeric_limits<float>::infinity()};
  float y_max{ std::numeric_limits<float>::infinity()};
  float z_min{-std::numeric_limits<float>::infinity()};
  float z_max{ std::numeric_limits<float>::infinity()};

  inline bool contains(float x, float y, float z) const noexcept
  {
    return x >= x_min && x <= x_max
        && y >= y_min && y <= y_max
        && z >= z_min && z <= z_max;
  }
};

class BoxFilter
{
public:
  // Parse the `box_filter:` sub-node from a `driver:` YAML node. Missing /
  // empty node => filter stays disabled (no-op apply()).
  void init(const YAML::Node& driver_config)
  {
    if (!driver_config["box_filter"]) return;
    const YAML::Node node = driver_config["box_filter"];

    bool enable = false;
    if (node["enable"]) enable = node["enable"].as<bool>();

    if (node["remove_inside"])
    {
      for (const auto& b : node["remove_inside"])
      {
        CropBox box;
        if (b["name"])  box.name  = b["name"].as<std::string>();
        else            box.name  = "box";
        if (b["x_min"]) box.x_min = b["x_min"].as<float>();
        if (b["x_max"]) box.x_max = b["x_max"].as<float>();
        if (b["y_min"]) box.y_min = b["y_min"].as<float>();
        if (b["y_max"]) box.y_max = b["y_max"].as<float>();
        if (b["z_min"]) box.z_min = b["z_min"].as<float>();
        if (b["z_max"]) box.z_max = b["z_max"].as<float>();
        if (box.x_min > box.x_max || box.y_min > box.y_max || box.z_min > box.z_max)
        {
          RS_WARNING << "box_filter: '" << box.name
                     << "' has inverted bounds; skipping." << RS_REND;
          continue;
        }
        remove_boxes_.push_back(box);
      }
    }

    enabled_ = enable && !remove_boxes_.empty();
    if (enabled_)
    {
      RS_INFO << "box_filter: enabled with " << remove_boxes_.size()
              << " remove-box(es)." << RS_REND;
      for (const auto& b : remove_boxes_)
      {
        RS_INFO << "  - " << b.name
                << " x[" << b.x_min << "," << b.x_max << "]"
                << " y[" << b.y_min << "," << b.y_max << "]"
                << " z[" << b.z_min << "," << b.z_max << "]" << RS_REND;
      }
    }
    else if (enable)
    {
      RS_WARNING << "box_filter: enable=true but no remove_inside boxes; disabled."
                 << RS_REND;
    }
  }

  inline bool enabled() const noexcept { return enabled_; }

  // - msg.is_dense == true: compacts the cloud (erases filtered points,
  //   updates width/height).
  // - msg.is_dense == false: sets x/y/z to NaN so the (height x width) grid
  //   is preserved (matches dense_points=false convention; ring/timestamp
  //   stay intact). Points already at NaN are left alone.
  void apply(LidarPointCloudMsg& msg) const
  {
    if (!enabled_) return;

    if (msg.is_dense)
    {
      auto& pts = msg.points;
      auto it = std::remove_if(
          pts.begin(), pts.end(),
          [this](const typename LidarPointCloudMsg::PointT& p)
          { return insideAny(p.x, p.y, p.z); });
      pts.erase(it, pts.end());
      msg.width  = static_cast<uint32_t>(pts.size());
      msg.height = 1;
    }
    else
    {
      const float nan = std::numeric_limits<float>::quiet_NaN();
      for (auto& p : msg.points)
      {
        if (std::isnan(p.x)) continue;
        if (insideAny(p.x, p.y, p.z))
        {
          p.x = nan; p.y = nan; p.z = nan;
        }
      }
    }
  }

private:
  inline bool insideAny(float x, float y, float z) const noexcept
  {
    for (const auto& b : remove_boxes_) if (b.contains(x, y, z)) return true;
    return false;
  }

  bool enabled_{false};
  std::vector<CropBox> remove_boxes_;
};

}  // namespace lidar
}  // namespace robosense
