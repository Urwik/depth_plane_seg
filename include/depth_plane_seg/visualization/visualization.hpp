#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "segmentation/base_plane.hpp"

namespace utils {

    void visualizeSegmentation(const std::vector<pcl::PointIndices> clusters, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Segmentation Visualizer"));
        
        viewer->setBackgroundColor(0, 0, 0);

        // Create two viewports        
        int left(0), right(1);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, left);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, right);
        
        // Add point cloud to the first viewport
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", left);

        // Add clusters to the second viewport
        for (size_t i = 0; i < clusters.size(); i++) {

            float R = (float) (rand() % 255) / 255.0;
            float G = (float) (rand() % 255) / 255.0;
            float B = (float) (rand() % 255) / 255.0;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t j = 0; j < clusters[i].indices.size(); j++) {
                cluster->points.push_back(cloud->points[clusters[i].indices[j]]);
            }
            viewer->addPointCloud<pcl::PointXYZ>(cluster, "cluster" + std::to_string(i), right);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "cluster" + std::to_string(i));
        }

        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
    }

    void visualizeSegmentationWithNormals(const std::vector<pcl::PointIndices> clusters, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals) {
        
        if (normals->points.size() == 0) {
            std::cerr << "No normals to visualize" << std::endl;
            return;
        }

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Segmentation Visualizer"));
        
        viewer->setBackgroundColor(0, 0, 0);

        // Create two viewports        
        int left(0), right(1);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, left);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, right);
        
        // Add point cloud to the first viewport
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", left);

        // Add clusters to the second viewport
        for (size_t i = 0; i < clusters.size(); i++) {

            float R = (float) (rand() % 255) / 255.0;
            float G = (float) (rand() % 255) / 255.0;
            float B = (float) (rand() % 255) / 255.0;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t j = 0; j < clusters[i].indices.size(); j++) {
                cluster->points.push_back(cloud->points[clusters[i].indices[j]]);
            }
            viewer->addPointCloud<pcl::PointXYZ>(cluster, "cluster" + std::to_string(i), right);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "cluster" + std::to_string(i));
        }

        // Add normals to the first viewport
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals", left);

        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
    }

    void simpleVisualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Simple Visualizer"));
        
        viewer->setBackgroundColor(0, 0, 0);

        viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
    }

    void visualizePlanes(const std::vector<Plane> &planos) {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Planes Visualizer"));

        viewer->setBackgroundColor(1,1,1);

        for(size_t i=0; i<planos.size(); i++) {
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>);

            for(const auto& point : planos[i].edges) {
                pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
                edge_points->points.push_back(pcl_point);
            }


            float R = (float) (rand() % 255) / 255.0;
            float G = (float) (rand() % 255) / 255.0;
            float B = (float) (rand() % 255) / 255.0;

            viewer->addPointCloud<pcl::PointXYZ>(edge_points, "edge_points" + std::to_string(i));
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "edge_points" + std::to_string(i));
            viewer->addPolygon<pcl::PointXYZ>(edge_points, R, G, B, "plane" + std::to_string(i));
        }

        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
    }

    void fullDisplay(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::vector<pcl::PointIndices>& clusters, const std::vector<Plane> &planos, const std::vector<Plane> &map_planes = {}) {
        
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Full Display"));
        viewer->loadCameraParameters("/home/mauro/amr_ws/src/depth_plane_seg/config/cam_params.cam");

        // Create four viewports
        int tl(0), tr(1), bl(2), br(3);
        viewer->createViewPort(0.0, 0.5, 0.5, 1.0, tl);
        viewer->createViewPort(0.5, 0.5, 1.0, 1.0, tr);
        viewer->createViewPort(0.0, 0.0, 0.5, 0.5, bl);
        viewer->createViewPort(0.5, 0.0, 1.0, 0.5, br);
        
        viewer->setBackgroundColor(1,1,1);


        // Add original cloud to the first viewport
        viewer->addText("Input Cloud", 10, 10, "tl_text", tl);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "input_cloud", tl);


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
            

            viewer->addPointCloud<pcl::PointXYZ>(cluster, "cluster" + std::to_string(i), tr);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "cluster" + std::to_string(i), tr);
        }

        // Add extracted planes polygons to the third viewport
        for(size_t i=0; i<planos.size(); i++) {
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>);

            for(const auto& point : planos[i].edges) {
                pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
                edge_points->points.push_back(pcl_point);
            }


            float R = (float) (rand() % 255) / 255.0;
            float G = (float) (rand() % 255) / 255.0;
            float B = (float) (rand() % 255) / 255.0;

            viewer->addPointCloud<pcl::PointXYZ>(edge_points, "edge_points" + std::to_string(i), bl);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "edge_points" + std::to_string(i), bl);
            viewer->addPolygon<pcl::PointXYZ>(edge_points, R, G, B, "plane" + std::to_string(i), bl);
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

            viewer->addPointCloud<pcl::PointXYZ>(edge_points, "edge_points" + std::to_string(i), bl);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, "edge_points" + std::to_string(i), bl);
            viewer->addPolygon<pcl::PointXYZ>(edge_points, R, G, B, "plane" + std::to_string(i), bl);
        }

        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
    }
} // namespace utils

#endif