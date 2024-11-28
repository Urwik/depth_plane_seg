#ifndef REGROW_PLANE_SEGMENTER_HPP
#define REGROW_PLANE_SEGMENTER_HPP

#include "segmentation/base_segmenter.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/features/integral_image_normal.h>

class ReGrowPlaneSegmenter : public BaseSegmenter {

public:
    ReGrowPlaneSegmenter();
    ~ReGrowPlaneSegmenter();

    using BaseSegmenter::setInputCloud; // Inherit the base class setInputCloud function
    void setInputCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

    pcl::PointCloud<pcl::Normal>::Ptr getNormals();

    void segment() override;

private:
    pcl::PointCloud<pcl::Normal>::Ptr normals;
};

    ReGrowPlaneSegmenter::ReGrowPlaneSegmenter() {
        this->normals.reset(new pcl::PointCloud<pcl::Normal>);
    }

    ReGrowPlaneSegmenter::~ReGrowPlaneSegmenter() {
        this->normals.reset();
    }


    pcl::PointCloud<pcl::Normal>::Ptr
    ReGrowPlaneSegmenter::getNormals(){
        return this->normals;
    }

    void
    ReGrowPlaneSegmenter::segment(){


        pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

        // Compute normals if not present
        if(this->normals->points.size() == 0) {
            std::cout << "Computing normals" << std::endl;


            // --> Normal estimation
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
            normal_estimator.setSearchMethod (tree);
            normal_estimator.setInputCloud (this->input_cloud);
            normal_estimator.setKSearch (50);
            normal_estimator.compute (*this->normals);

            // --> Integral image normal estimation
            // pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            // ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
            // ne.setMaxDepthChangeFactor(0.001f);
            // ne.setNormalSmoothingSize(10.0f);
            // ne.setInputCloud(this->input_cloud);
            // ne.compute(*this->normals);
        }   
        
        std::cout << "Normals size: " << this->normals->points.size() << std::endl;

        // pcl::IndicesPtr indices (new std::vector <int>);
        // pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*this->input_cloud, *this->input_cloud, *indices);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setSearchMethod (tree);
        reg.setInputCloud(this->input_cloud);
        // reg.setIndices (indices);
        reg.setMinClusterSize (1000);
        reg.setMaxClusterSize (1000000);
        reg.setNumberOfNeighbours (30);
        reg.setSmoothModeFlag (true); // Depends on the input cloud
        reg.setInputNormals (this->normals);
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (1.0);

        reg.extract (this->clusters);
    }

    void
    ReGrowPlaneSegmenter::setInputCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud){

        // Split the input cloud in an efficient way
        this->normals->points.reserve(cloud->points.size());
        std::transform(cloud->points.begin(), cloud->points.end(), std::back_inserter(normals->points), [](const pcl::PointNormal& pn){
            return pcl::Normal(pn.normal_x, pn.normal_y, pn.normal_z, pn.curvature);
        });

        this->input_cloud->points.reserve(cloud->points.size());
        std::transform(cloud->points.begin(), cloud->points.end(), std::back_inserter(input_cloud->points), [](const pcl::PointNormal& pn){
            return pcl::PointXYZ(pn.x, pn.y, pn.z);
        });

    }


#endif