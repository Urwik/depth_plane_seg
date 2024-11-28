#ifndef RGBD_ALIGNMENT_HPP
#define RGBD_ALIGNMENT_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace fs=std::filesystem;

/**
 * @brief Convert a YAML node to an Eigen Matrix3f
 * Reads the extrisics from a YAML node and returns an Eigen Matrix3f with R and T
 */
Eigen::Matrix4f yaml2EigenExstrinsics(YAML::Node config, std::string camera)
{
    Eigen::Matrix3f rotation;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rotation(i, j) = config[camera]["rotation"][i*3 + j].as<float>();
        }
    }

    Eigen::Vector3f translation;
    for (int i = 0; i < 3; i++)
        translation(i) = config[camera]["translation"][i].as<float>();

    Eigen::Matrix4f extrinsics = Eigen::Matrix4f::Identity();
    extrinsics.block<3, 3>(0, 0) = rotation;
    extrinsics.block<3, 1>(0, 3) = translation;
    return extrinsics;
}

/**
 * @brief Class to align RGB and Depth images
 */
class RGBDAlignment {

    struct Intrinsics {
        float fx = 0.0f;
        float fy = 0.0f;
        float cx = 0.0f;
        float cy = 0.0f;
    };

    struct Extrinsics {
        Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
        Eigen::Vector3f translation = Eigen::Vector3f::Zero();
    };

public:
    RGBDAlignment();
    RGBDAlignment(const fs::path& rgb_image, const fs::path& depth_image);
    RGBDAlignment(const cv::Mat& rgb_image, const cv::Mat& depth_image);
    ~RGBDAlignment();

    void setRGBImage(const fs::path& rgb_image);
    void setDepthImage(const fs::path& depth_image);
    void setRGBImage(const cv::Mat& rgb_image);
    void setDepthImage(const cv::Mat& depth_image);

    void setDepthIntrinsics(const float cx, const float cy, const float fx, const float fy);
    void setRGBIntrinsics(const float cx, const float cy, const float fx, const float fy);
    void setRotation(const Eigen::Matrix3f& rotation);
    void setTranslation(const Eigen::Vector3f& translation);
    void setDepthScale(float scale);

    void setParams(const fs::path& params_file = "");
    void setParams(const YAML::Node& params);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPoinCloud();

    void run();

private:
    cv::Mat rgb_image, depth_image;
    
    Intrinsics depth_intrinsics, rgb_intrinsics;
    Extrinsics depth_to_color;
    float depthScale;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

};

RGBDAlignment::RGBDAlignment() {
    this->cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    this->rgb_image = cv::Mat();
    this->depth_image = cv::Mat();
    this->depthScale = 1.0f;
}

RGBDAlignment::RGBDAlignment(const fs::path& rgb_image, const fs::path& depth_image) {
    RGBDAlignment();
    
    if(!fs::exists(rgb_image) || !fs::exists(depth_image)) {
        std::cerr << "Could not open or find the images." << std::endl;
        return;
    }

    this->setRGBImage(rgb_image);
    this->setDepthImage(depth_image);
}

RGBDAlignment::RGBDAlignment(const cv::Mat& rgb_image, const cv::Mat& depth_image) {
    RGBDAlignment();

    if(rgb_image.empty() || depth_image.empty()) {
        std::cerr << "Images are empty" << std::endl;
        return;
    }

    this->setRGBImage(rgb_image);
    this->setDepthImage(depth_image);
}


RGBDAlignment::~RGBDAlignment() {
    this->rgb_image.release();
    this->depth_image.release();
}

/**
 * @brief Set the RGB image from a file path
 * @param rgb_image Path to the RGB image
 */
void RGBDAlignment::setRGBImage(const fs::path& rgb_image) {
    this->rgb_image = cv::imread(rgb_image.string(), cv::IMREAD_COLOR);
}

/**
 * @brief Set the Depth image from a file path
 * @param depth_image Path to the Depth image
 */
void RGBDAlignment::setDepthImage(const fs::path& depth_image) {
    this->depth_image = cv::imread(depth_image.string(), cv::IMREAD_UNCHANGED);
}

/**
 * @brief Set the RGB image from a cv::Mat
 * @param rgb_image cv::Mat with the RGB image
 */
void RGBDAlignment::setRGBImage(const cv::Mat& rgb_image) {
    this->rgb_image = rgb_image;
}

void RGBDAlignment::setDepthImage(const cv::Mat& depth_image) {
    this->depth_image = depth_image;
}


void RGBDAlignment::setDepthIntrinsics(const float cx, const float cy, const float fx, const float fy) {
    this->depth_intrinsics.cx = cx;
    this->depth_intrinsics.cy = cy;
    this->depth_intrinsics.fx = fx;
    this->depth_intrinsics.fy = fy;
}

void RGBDAlignment::setRGBIntrinsics(const float cx, const float cy, const float fx, const float fy) {
    this->rgb_intrinsics.cx = cx;
    this->rgb_intrinsics.cy = cy;
    this->rgb_intrinsics.fx = fx;
    this->rgb_intrinsics.fy = fy;
}

void RGBDAlignment::setRotation(const Eigen::Matrix3f& rotation) {
    this->depth_to_color.rotation = rotation;
}

void RGBDAlignment::setTranslation(const Eigen::Vector3f& translation) {
    this->depth_to_color.translation = translation;
}

void RGBDAlignment::setDepthScale(float scale) {
    this->depthScale = scale;
}


void RGBDAlignment::setParams(const fs::path& params_file) {

    if(params_file == "" || !fs::exists(params_file)) {
        std::cerr << "No params file provided or not exists, loading default params file" << std::endl;
        const YAML::Node config = YAML::LoadFile("/home/mauro/amr_ws/src/depth_plane_seg/include/rgbd_alignment/params.yaml");    
        this->setParams(config);
    }
    else {
        const YAML::Node config = YAML::LoadFile(params_file.string());
        this->setParams(config);
    }

}

void RGBDAlignment::setParams(const YAML::Node& params) {
    this->depth_intrinsics.fx = params["intrinsics"]["depth"]["fx"].as<float>();
    this->depth_intrinsics.fy = params["intrinsics"]["depth"]["fy"].as<float>();
    this->depth_intrinsics.cx = params["intrinsics"]["depth"]["cx"].as<float>();
    this->depth_intrinsics.cy = params["intrinsics"]["depth"]["cy"].as<float>();

    this->rgb_intrinsics.fx = params["intrinsics"]["color"]["fx"].as<float>();
    this->rgb_intrinsics.fy = params["intrinsics"]["color"]["fy"].as<float>();
    this->rgb_intrinsics.cx = params["intrinsics"]["color"]["cx"].as<float>();
    this->rgb_intrinsics.cy = params["intrinsics"]["color"]["cy"].as<float>();

    this->depth_to_color.rotation = yaml2EigenExstrinsics(params, "depth_to_color").block<3, 3>(0, 0);
    this->depth_to_color.translation = yaml2EigenExstrinsics(params, "depth_to_color").block<3, 1>(0, 3);

    this->depthScale = params["depth_scale"].as<float>();
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBDAlignment::getPoinCloud() {
    return this->cloud;
}


void RGBDAlignment::run() {


    this->cloud->width = this->rgb_image.cols;
    this->cloud->height = this->rgb_image.rows;
    this->cloud->is_dense = false;  // Depth images can have NaNs
    this->cloud->points.resize(this->cloud->width * this->cloud->height);

    // Iterate over each pixel in the depth image
    for (int v = 0; v < this->depth_image.rows; ++v) {
        for (int u = 0; u < this->depth_image.cols; ++u) {
            // Get the depth value for this pixel
            float depth = this->depth_image.at<uint16_t>(v, u) * this->depthScale;
            
            // Check for invalid measurements
            if (depth > 0) {

                // Convert (u, v, depth) to camera space coordinates in the depth camera frame
                Eigen::Vector3f point3D_depth;
                point3D_depth.z() = depth;
                point3D_depth.x() = (u - this->depth_intrinsics.cx) * depth / this->depth_intrinsics.fx;
                point3D_depth.y() = (v - this->depth_intrinsics.cy) * depth / this->depth_intrinsics.fy;


                // Transform point from depth camera frame to RGB camera frame using extrinsics
                Eigen::Vector3f point3D_rgb = (this->depth_to_color.rotation * point3D_depth) + this->depth_to_color.translation;

                // Project to RGB image plane using RGB camera intrinsics
                int x_rgb = static_cast<int>((point3D_rgb.x() * this->rgb_intrinsics.fx / point3D_rgb.z()) + this->rgb_intrinsics.cx);
                int y_rgb = static_cast<int>((point3D_rgb.y() * this->rgb_intrinsics.fy / point3D_rgb.z()) + this->rgb_intrinsics.cy);

                // Only add point if it projects within the RGB image bounds
                if (x_rgb >= 0 && x_rgb < this->rgb_image.cols && y_rgb >= 0 && y_rgb < this->rgb_image.rows) {
                    pcl::PointXYZRGB& point = cloud->at(x_rgb, y_rgb);
                    point.x = point3D_rgb.x();
                    point.y = point3D_rgb.y();
                    point.z = point3D_rgb.z();

                    // Get the RGB color from the RGB image
                    cv::Vec3b color = this->rgb_image.at<cv::Vec3b>(y_rgb, x_rgb);
                    point.b = color[0];
                    point.g = color[1];
                    point.r = color[2];
                }
            }
            // If measure is invalid set the point to NaN
            else
            {
                // Set the point to NaN if there is no depth value
                pcl::PointXYZRGB& point = cloud->at(u,v);
                point.x = std::numeric_limits<float>::quiet_NaN();
                point.y = std::numeric_limits<float>::quiet_NaN();
                point.z = std::numeric_limits<float>::quiet_NaN();
                point.r = 0;
                point.g = 0;
                point.b = 0;
            }
        }
    }
}

#endif