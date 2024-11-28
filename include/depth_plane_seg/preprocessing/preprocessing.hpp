#ifndef PREPROCESSING_HPP
#define PREPROCESSING_HPP

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>


/**
 * @brief Class for preprocessing point cloud data
 */
class Preprocessing {
    
public:
    Preprocessing();
    Preprocessing(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    ~Preprocessing();

    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void voxelize(float leaf_size);
    void removeStatisticalOutliers(int mean_k, float std_dev);
    void removeRadiusOutliers(float radius, int min_neighbors);
    void removeNaNs();
    void rangeFilter(float min_range, float max_range);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

Preprocessing::Preprocessing() {
    this->cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

Preprocessing::Preprocessing(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    this->cloud = cloud;
}

Preprocessing::~Preprocessing() {
    this->cloud.reset();
}

/**
 * @brief Set input point cloud data
 */
void Preprocessing::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    this->cloud = cloud;
}

/**
 * @brief Voxel grid filter homogeneous in all dimensions
 * @param leaf_size Voxel size
 */
void Preprocessing::voxelize(float leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(this->cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*this->cloud);
}


/**
 * @brief Remove outliers using mean and standard deviation
 * @param mean_k Number of neighbors to analyze
 * @param std_dev Standard deviation multiplier
 */
void Preprocessing::removeStatisticalOutliers(int mean_k, float std_dev) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(this->cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_dev);
    sor.filter(*this->cloud);
}

/**
 * @brief Remove outliers using radius and minimum neighbors
 * @param radius Radius to search for neighbors
 * @param min_neighbors Minimum number of neighbors
 */
void Preprocessing::removeRadiusOutliers(float radius, int min_neighbors) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(this->cloud);
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(min_neighbors);
    ror.filter(*this->cloud);
}

/**
 * @brief Remove NaN values from point cloud
 */
void Preprocessing::removeNaNs() {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*this->cloud, *this->cloud, indices);
}

/**
 * @brief Filter point cloud data based on range
 * @param min_range Minimum range
 * @param max_range Maximum range
 */
void Preprocessing::rangeFilter(float min_range, float max_range) {
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, min_range)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, max_range)));

    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(this->cloud);
    condrem.setKeepOrganized(true);
    condrem.filter(*this->cloud);
}


#endif