#ifndef STREAM_VIEWER_HPP
#define STREAM_VIEWER_HPP

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "segmentation/base_plane.hpp"

class StreamViewer {

    public:
        StreamViewer();
        ~StreamViewer();

        void update(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::vector<pcl::PointIndices>& clusters, const std::vector<Plane> &planes, const std::vector<Plane> &map_planes = {});
        void update(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices>& clusters, const std::vector<Plane> &planes, const std::vector<Plane> &map_planes = {});
        void spinOnce();

    private:
        pcl::visualization::PCLVisualizer::Ptr viewer;
        int tl, tr, bl, br;
};


StreamViewer::StreamViewer() {
    this->viewer.reset(new pcl::visualization::PCLVisualizer("Stream Viewer"));
    this->viewer->loadCameraParameters("/home/fran/workspaces/isec/jazzy_ws/src/rgbd_plane_mapping/examples/pcd/cam_params.cam");
    
    this->viewer->setBackgroundColor(0, 0, 0);

    // Create four viewports
    this->tl = int(0); this->tr = int(1); this->bl = int(2); this->br = int(3);
    this->viewer->createViewPort(0.0, 0.5, 0.5, 1.0, this->tl);
    this->viewer->createViewPort(0.5, 0.5, 1.0, 1.0, this->tr);
    this->viewer->createViewPort(0.0, 0.0, 0.5, 0.5, this->bl);
    this->viewer->createViewPort(0.5, 0.0, 1.0, 0.5, this->br);
}

StreamViewer::~StreamViewer() {
    this->viewer.reset();
}

void
StreamViewer::update(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::vector<pcl::PointIndices>& clusters, const std::vector<Plane> &planes, const std::vector<Plane> &map_planes ) {

    this->viewer->removeAllPointClouds();
    this->viewer->removeAllShapes();

    // Add original cloud to the first viewport   
    this->viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud", this->tl);


    // Create temporal cloud to add segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_xyz);

    // Add segmentation to the second viewport
    for (size_t i = 0; i < clusters.size(); i++) {

        float R = (float) (rand() % 255) / 255.0;
        float G = (float) (rand() % 255) / 255.0;
        float B = (float) (rand() % 255) / 255.0;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (size_t j = 0; j < clusters[i].indices.size(); j++) {
            cluster->points.push_back(cloud_xyz->points[clusters[i].indices[j]]);
        }
        

        this->viewer->addPointCloud<pcl::PointXYZ>(cluster, "cluster" + std::to_string(i), this->tr);
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "cluster" + std::to_string(i), this->tr);
    }

    // Add extracted planes polygons to the third viewport
    for(size_t i=0; i<planes.size(); i++) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>);

        for(const auto& point : planes[i].edges) {
            pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
            edge_points->points.push_back(pcl_point);
        }


        float R = (float) (rand() % 255) / 255.0;
        float G = (float) (rand() % 255) / 255.0;
        float B = (float) (rand() % 255) / 255.0;

        this->viewer->addPointCloud<pcl::PointXYZ>(edge_points, "edge_points" + std::to_string(i), this->bl);
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "edge_points" + std::to_string(i), this->bl);
        this->viewer->addPolygon<pcl::PointXYZ>(edge_points, R, G, B, "plane" + std::to_string(i), this->bl);
    }


    
    // Add map planes to the fourth viewport
    for(size_t i=0; i<map_planes.size(); i++) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>);

        for(const auto& point : map_planes[i].edges) {
            pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
            edge_points->points.push_back(pcl_point);
        }


        float R = (float) (rand() % 255) / 255.0;
        float G = (float) (rand() % 255) / 255.0;
        float B = (float) (rand() % 255) / 255.0;

        this->viewer->addPointCloud<pcl::PointXYZ>(edge_points, "map_edge_points" + std::to_string(i), this->br);
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "map_edge_points" + std::to_string(i), this->br);
        this->viewer->addPolygon<pcl::PointXYZ>(edge_points, R, G, B, "map_plane" + std::to_string(i), this->br);
    }
}

void
StreamViewer::update(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices>& clusters, const std::vector<Plane> &planes, const std::vector<Plane> &map_planes) {

    this->viewer->removeAllPointClouds();
    this->viewer->removeAllShapes();

    // Add original cloud to the first viewport   
    this->viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", this->tl);

    // Add segmentation to the second viewport
    for (size_t i = 0; i < clusters.size(); i++) {

        float R = (float) (rand() % 255) / 255.0;
        float G = (float) (rand() % 255) / 255.0;
        float B = (float) (rand() % 255) / 255.0;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (size_t j = 0; j < clusters[i].indices.size(); j++) {
            cluster->points.push_back(cloud->points[clusters[i].indices[j]]);
        }
        

        this->viewer->addPointCloud<pcl::PointXYZ>(cluster, "cluster" + std::to_string(i), this->tr);
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "cluster" + std::to_string(i), this->tr);
    }

    // Add extracted planes polygons to the third viewport
    for(size_t i=0; i<planes.size(); i++) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>);

        for(const auto& point : planes[i].edges) {
            pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
            edge_points->points.push_back(pcl_point);
        }


        float R = (float) (rand() % 255) / 255.0;
        float G = (float) (rand() % 255) / 255.0;
        float B = (float) (rand() % 255) / 255.0;

        this->viewer->addPointCloud<pcl::PointXYZ>(edge_points, "edge_points" + std::to_string(i), this->bl);
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "edge_points" + std::to_string(i), this->bl);
        this->viewer->addPolygon<pcl::PointXYZ>(edge_points, R, G, B, "plane" + std::to_string(i), this->bl);
    }


    
    // Add map planes to the fourth viewport
    for(size_t i=0; i<map_planes.size(); i++) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>);

        for(const auto& point : map_planes[i].edges) {
            pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
            edge_points->points.push_back(pcl_point);
        }


        float R = (float) (rand() % 255) / 255.0;
        float G = (float) (rand() % 255) / 255.0;
        float B = (float) (rand() % 255) / 255.0;

        this->viewer->addPointCloud<pcl::PointXYZ>(edge_points, "map_edge_points" + std::to_string(i), this->br);
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "map_edge_points" + std::to_string(i), this->br);
        this->viewer->addPolygon<pcl::PointXYZ>(edge_points, R, G, B, "map_plane" + std::to_string(i), this->br);
    }
}

void StreamViewer::spinOnce() {
    this->viewer->spinOnce(200);
}

#endif