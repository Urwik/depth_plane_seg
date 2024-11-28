#ifndef BASE_SEGMENTER_HPP
#define BASE_SEGMENTER_HPP

#include <iostream>
#include <map>
#include <filesystem>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <yaml-cpp/yaml.h>

#include "timer.hpp"
#include "base_plane.hpp"



namespace fs = std::filesystem;

/**
 * @brief Base class for segmentation
 */
class BaseSegmenter {
    
    public:
        BaseSegmenter();
        ~BaseSegmenter();
    
        void setInputCloud(fs::path  cloud_path);
        void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void setParams(fs::path params_path);
        void setCloudId(const std::string id);

        virtual void segment();

        YAML::Node getParams();
        pcl::PointCloud<pcl::PointXYZ>::Ptr getInputCloud();
        std::vector<pcl::PointIndices> getClusters( bool filter_nan = false);
        virtual std::vector<Plane> getPlanes();

        void saveSegCloud();

    protected:

        virtual void computePlaneEdges(const std::vector<Eigen::Vector4f> &coeffs = std::vector<Eigen::Vector4f>());
    
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
        std::string cloud_id;
        std::vector<pcl::PointIndices> clusters;
        std::vector<Plane> planes;
        YAML::Node params;
        Timer timer;
};


BaseSegmenter::BaseSegmenter() {
    this->input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->clusters.clear();
    this->planes.clear();
}

BaseSegmenter::~BaseSegmenter() {
    this->input_cloud.reset();
    this->clusters.clear();
    this->planes.clear();
}

void
BaseSegmenter::setCloudId(const std::string id){
    this->cloud_id = id;
}

void
BaseSegmenter::segment(){
    std::cout << "Segmentation not implemented, this is BaseSegmenter segment function" << std::endl;
}

void 
BaseSegmenter::setParams(fs::path params_path) {
    try {
        this->params = YAML::LoadFile(params_path.string());
    } catch (YAML::BadFile& e) {
        std::cerr << "Params file not found" << std::endl;
    }
}


void 
BaseSegmenter::setInputCloud(fs::path cloud_path){
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile(cloud_path, cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *this->input_cloud);
}


void
BaseSegmenter::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::copyPointCloud(*cloud, *this->input_cloud);
}


void
BaseSegmenter::computePlaneEdges(const std::vector<Eigen::Vector4f> &coeffs){

#ifdef DEBUG
    std::cout << "computePlaneEdges from BaseSegmenter..." << std::endl;
#endif

    pcl::PointIndices::Ptr plane_indices (new pcl::PointIndices);
    

    for (size_t i = 0; i < this->clusters.size(); i++) {
        plane_indices->indices.clear();
        *plane_indices = this->clusters[i];

    Plane plane_model;
    if(coeffs.empty()){
        plane_model = Plane(this->input_cloud, plane_indices);    
    }
    else{
        plane_model = Plane(this->input_cloud, plane_indices, coeffs[i]);
    }

#ifdef DEBUG
        std::cout << "\tplane_model.center" << plane_model.center << std::endl;
        std::cout << "\tplane_model.coeffs" << plane_model.coeffs.transpose() << std::endl;

        for (const Eigen::Vector3f &edge : plane_model.edges) {
            std::cout << "\t\tedge: " << edge.transpose() << std::endl;
        }

#endif
        this->planes.push_back(plane_model);
    }

}


std::vector<Plane>
BaseSegmenter::getPlanes(){

    if (this->planes.empty()) {
        this->computePlaneEdges();
    }

    return this->planes;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
BaseSegmenter::getInputCloud(){
    return this->input_cloud;
}

YAML::Node
BaseSegmenter::getParams(){
    return this->params;
}

std::vector<pcl::PointIndices>
BaseSegmenter::getClusters(bool filter_nan){

    if (filter_nan) {
        std::vector<pcl::PointIndices> filtered_clusters;
        for (const pcl::PointIndices &cluster : this->clusters) {
            pcl::PointIndices filtered_cluster;
            for (const int &index : cluster.indices) {
                if (!std::isnan(this->input_cloud->points[index].x) && !std::isnan(this->input_cloud->points[index].y) && !std::isnan(this->input_cloud->points[index].z)) {
                    filtered_cluster.indices.push_back(index);
                }
            }
            filtered_clusters.push_back(filtered_cluster);
        }
        return filtered_clusters;
    }
    
    return this->clusters;
}


void
BaseSegmenter::saveSegCloud() {


    pcl::PointCloud<pcl::PointXYZRGB> seg_cloud;
    seg_cloud.width = this->input_cloud->width;
    seg_cloud.height = this->input_cloud->height;
    seg_cloud.is_dense = false;
    seg_cloud.points.assign(seg_cloud.width * seg_cloud.height, pcl::PointXYZRGB());

    for (const pcl::PointIndices &cluster : this->clusters) {

        const int R = rand() % 255;
        const int G = rand() % 255;
        const int B = rand() % 255;
        
        for (const int &index : cluster.indices) {
            seg_cloud.points[index].x = this->input_cloud->points[index].x;
            seg_cloud.points[index].y = this->input_cloud->points[index].y;
            seg_cloud.points[index].z = this->input_cloud->points[index].z;
            seg_cloud.points[index].r = R;
            seg_cloud.points[index].g = G;
            seg_cloud.points[index].b = B;
        }
    }

    fs::path output_path = this->params["output_dir"].as<std::string>();

    pcl::io::savePCDFileBinary(output_path / this->cloud_id, seg_cloud);

}

#endif