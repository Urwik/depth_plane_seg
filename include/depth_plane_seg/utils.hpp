#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>
#include <unordered_set>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

namespace utils {

pcl::PointIndices::Ptr removeIndices(pcl::PointIndices::Ptr& origin, const pcl::PointIndices::Ptr& removeFromOrigin) {    
  pcl::PointIndices::Ptr newIndices(new pcl::PointIndices);
  newIndices->indices = origin->indices;

  // Convert removeFromOrigin to an unordered_set for fast lookups
  std::unordered_set<int> elementsSet(removeFromOrigin->indices.begin(), removeFromOrigin->indices.end());

  // Use std::remove_if to move elements to be removed to the end of the vector
  auto newEnd = std::remove_if(newIndices->indices.begin(), newIndices->indices.end(), [&elementsSet](int value) {
      return elementsSet.find(value) != elementsSet.end();
  });

  // Erase the removed elements
  newIndices->indices.erase(newEnd, newIndices->indices.end());

  return newIndices;
}


void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size) {
  
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloud);

}


std::map<std::string, std::string> yaml2ini(const YAML::Node& yaml) {
    std::map<std::string, std::string> ini;
    for (auto it = yaml.begin(); it != yaml.end(); ++it) {
        ini[it->first.as<std::string>()] = it->second.as<std::string>();
    }
    return ini;
}

YAML::Node ini2yaml(const std::map<std::string, std::string>& ini) {
    YAML::Node yaml;
    for (auto it = ini.begin(); it != ini.end(); ++it) {
        yaml[it->first] = it->second;
    }
    return yaml;
}

} // namespace utils

#endif