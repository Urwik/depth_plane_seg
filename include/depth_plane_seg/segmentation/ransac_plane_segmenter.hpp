#ifndef RANSAC_PLANE_SEGMENTER_HPP
#define RANSAC_PLANE_SEGMENTER_HPP

#include "segmentation/base_segmenter.hpp"
#include "utils.hpp"

#include <pcl/segmentation/sac_segmentation.h>

class RansacPlaneSegmenter : public BaseSegmenter {
  public:
    RansacPlaneSegmenter(); 
    ~RansacPlaneSegmenter(); 

    void setMaxIterations(int maxIterations) { this->maxIterations_ = maxIterations; }
    void setDistanceThreshold(float distThreshold) { this->distThreshold_ = distThreshold; }
    void setOptimizeCoefficients(bool optimizeCoefs) { this->optimizeCoefs_ = optimizeCoefs; }


    void segment() override;

  private:
    bool optimizeCoefs_;
    float distThreshold_;
    int maxIterations_;
};


RansacPlaneSegmenter::RansacPlaneSegmenter() {
  this->maxIterations_ = 1000;
  this->distThreshold_ = 0.01;
  this->optimizeCoefs_ = true;
}

RansacPlaneSegmenter::~RansacPlaneSegmenter() {
  this->clusters.clear();
}

void
RansacPlaneSegmenter::segment() {

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> ransac;
  ransac.setInputCloud(this->input_cloud);
  ransac.setOptimizeCoefficients(this->optimizeCoefs_);
  ransac.setModelType(pcl::SACMODEL_PLANE);
  ransac.setMethodType(pcl::SAC_RANSAC);
  ransac.setMaxIterations(this->maxIterations_);
  ransac.setDistanceThreshold(this->distThreshold_);

  // Initialize the inliers and coefficients objects
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create a vector with all the indices
  pcl::PointIndices::Ptr remain_indices (new pcl::PointIndices);
  remain_indices->indices.resize(this->input_cloud->size());
  std::iota(remain_indices->indices.begin(), remain_indices->indices.end(), 0);
  std::cout << "Initial indices: " << remain_indices->indices.size() << std::endl;

  // Apply RANSAC until the number of remaining indices is less than 10% of the total
  while (remain_indices->indices.size() > 0.1 * this->input_cloud->points.size()) {
    ransac.setIndices(remain_indices);
    ransac.segment(*inliers, *coefficients);

    std::cout << "\tRemain indices: " << remain_indices->indices.size() << std::endl;

    *remain_indices = *utils::removeIndices(remain_indices, inliers);

    // Save inliers
    this->clusters.push_back(*inliers);
  }

}

#endif