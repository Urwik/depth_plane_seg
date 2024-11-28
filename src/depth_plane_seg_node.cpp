#include <iostream>
#include <filesystem>
#include <iomanip>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/polygon.hpp>


#include "preprocessing/preprocessing.hpp"
#include "custom_ros2_msgs/msg/polygon_array.hpp"    

// #include "segmentation/base_segmenter.hpp"
// #include "segmentation/ransac_plane_segmenter.hpp" 
// #include "segmentation/regrow_plane_segmenter.hpp"
#include "segmentation/peac_interface.hpp"
#include "visualization/visualization.hpp"
#include "visualization/stream_viewer.hpp"
// #include "segmentation/flood_fill/flood_fill.hpp" // TODO
// #include "data/dataloader.hpp" // For loading point clouds from a directory
#include "timer.hpp"


namespace fs = std::filesystem;


double total_seg_time = 0.0;


class DepthPlaneSegNode : public rclcpp::Node {

    
    public:
    DepthPlaneSegNode() : Node("depth_plane_seg_node") {
        
        this->segmenter = PeacSegmenter();
        this->segmenter.setParams("/home/mauro/amr_ws/src/depth_plane_seg/config/params.yaml");
        this->segmenter.loadParams();

        this->params = YAML::LoadFile("/home/mauro/amr_ws/src/depth_plane_seg/config/params.yaml");

        std::string camera_info_topic = this->params["camera_info_topic"].as<std::string>();
        std::string depth_image_topic = this->params["depth_image_topic"].as<std::string>();
        std::string planes_segmentation_topic = this->params["planes_seg_topic"].as<std::string>();

        this->camera_info_subscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic, 1, std::bind(&DepthPlaneSegNode::camera_info_callback, this, std::placeholders::_1));
        
        this->subscriber = this->create_subscription<sensor_msgs::msg::Image>(depth_image_topic, 1, std::bind(&DepthPlaneSegNode::depth_callback, this, std::placeholders::_1));
        
        this->publisher = this->create_publisher<custom_ros2_msgs::msg::PolygonArray>(planes_segmentation_topic, 10);

        this->process_thread = std::thread(&DepthPlaneSegNode::process, this);
    }

    ~DepthPlaneSegNode() {
        if (this->process_thread.joinable())
            this->process_thread.join();
    }

    private:

    /**
     * @brief Callback for the depth image message
     * @param msg Depth image message
     * @return void
     * @note This function is called every time a depth image is received and adds it to the queue
     */
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        
        std::lock_guard<std::mutex> lock(this->queue_mutex);
        this->depth_image_queue.push(msg);
        this->queue_cv.notify_one();
    }


    /**
     * @brief Callback for the camera info message
     * @param msg Camera info message
     * @return void
     * @note This function is called once to set the camera intrinsics
     */
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {

        if (msg->k.at(0) != 0.0) {
            this->depth_intrinsics = *msg;
            this->camera_info_received = true;
            this->camera_info_subscriber.reset();
#ifdef DEBUG
            std::cout << "Camera intrinsics received" << std::endl;
#endif
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Invalid camera intrinsics");
        }
    }


    /**
     * @brief Converts a depth image to a PCL point cloud
     * @param msg Depth image message
     * @param depthScale Depth scale factor
     * @return PCL point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    depth2pcl(const sensor_msgs::msg::Image::SharedPtr msg, const float depthScale = 1000.0f) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_xyz->width = msg->width;
    cloud_xyz->height = msg->height;
    cloud_xyz->is_dense = false;
    cloud_xyz->points.resize(cloud_xyz->width * cloud_xyz->height);

    double cx = this->depth_intrinsics.k.at(2);
    double cy = this->depth_intrinsics.k.at(5);
    double fx = this->depth_intrinsics.k.at(0);
    double fy = this->depth_intrinsics.k.at(4);

    cv::Mat depth_image(cv::Size(msg->width, msg->height), CV_16UC1, const_cast<unsigned char*>(msg->data.data()), msg->step);

    for (int v = 0; v < depth_image.rows; ++v) {
        for (int u = 0; u < depth_image.cols; ++u) {
            // Get the depth value for this pixel
            float depth = depth_image.at<uint16_t>(v, u) * depthScale;
            
            // Check for invalid measurements
            if (depth > 0) {

                pcl::PointXYZ& point = cloud_xyz->at(u,v);
                // Convert (u, v, depth) to camera space coordinates in the depth camera frame
                Eigen::Vector3f point3D_depth;
                point3D_depth.z() = depth;
                point3D_depth.x() = (u - cx) * depth / fx;
                point3D_depth.y() = (v - cy) * depth / fy;

                point.x = point3D_depth.x();
                point.y = point3D_depth.y();
                point.z = point3D_depth.z();

                // // Transform point from depth camera frame to RGB camera frame using extrinsics
                // Eigen::Vector3f point3D_rgb = (this->depth_to_color.rotation * point3D_depth) + this->depth_to_color.translation;

                // // Project to RGB image plane using RGB camera intrinsics
                // int x_rgb = static_cast<int>((point3D_rgb.x() * this->rgb_intrinsics.fx / point3D_rgb.z()) + this->rgb_intrinsics.cx);
                // int y_rgb = static_cast<int>((point3D_rgb.y() * this->rgb_intrinsics.fy / point3D_rgb.z()) + this->rgb_intrinsics.cy);

                // // Only add point if it projects within the RGB image bounds
                // if (x_rgb >= 0 && x_rgb < this->rgb_image.cols && y_rgb >= 0 && y_rgb < this->rgb_image.rows) {
                //     pcl::PointXYZRGB& point = cloud->at(x_rgb, y_rgb);
                //     point.x = point3D_rgb.x();
                //     point.y = point3D_rgb.y();
                //     point.z = point3D_rgb.z();

                //     // Get the RGB color from the RGB image
                //     cv::Vec3b color = this->rgb_image.at<cv::Vec3b>(y_rgb, x_rgb);
                //     point.b = color[0];
                //     point.g = color[1];
                //     point.r = color[2];
                // }
            }
            // If measure is invalid set the point to NaN
            else
            {
                // Set the point to NaN if there is no depth value
                pcl::PointXYZ& point = cloud_xyz->at(u,v);
                point.x = std::numeric_limits<float>::quiet_NaN();
                point.y = std::numeric_limits<float>::quiet_NaN();
                point.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    return cloud_xyz;
}


    /**
     * @brief Publishes the polygon array message
     * @return void
     * @note This function is called after the segmentation is done, and publish custom_ros2_msgs::msg::PolygonArray message
     */
    void publishPolygonArray() {
        
        std::vector<Plane> planes = this->segmenter.getPlanes();

        custom_ros2_msgs::msg::PolygonArray polygon_array;
        for (const Plane &plane : planes) {
            geometry_msgs::msg::Polygon polygon;
            for (const Eigen::Vector3f &edge : plane.edges) {
                geometry_msgs::msg::Point32 point;
                point.x = edge.x();
                point.y = edge.y();
                point.z = edge.z();
                polygon.points.push_back(point);
            }
            polygon_array.polygons.push_back(polygon);
        }
        this->publisher->publish(polygon_array);
    }


    /**
     * @brief Main loop of the node
     * @return void
     * @note This function is called in a loop to process the data
     */
    void process() {

        while (rclcpp::ok()) {
            if (!this->camera_info_received) {
                RCLCPP_INFO(this->get_logger(), "Waiting for camera info...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            else {
                break;
            }
        }
        

        while (rclcpp::ok()) {
#ifdef TIMER
            auto start = std::chrono::high_resolution_clock::now();
#endif

            std::unique_lock<std::mutex> lock(this->queue_mutex);
            this->queue_cv.wait(lock, [this] { return !this->depth_image_queue.empty(); });

#ifdef DEBUG
            std::cout << "\n## NEW IMAGE ##" << std::endl;
#endif  
            // Get the depth image from the queue, remove it from the queue and unlock the queue
            sensor_msgs::msg::Image::SharedPtr depth_image = this->depth_image_queue.front();
            this->depth_image_queue.pop();
            lock.unlock();            

            // Convert the depth image to a PCL point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz = this->depth2pcl(depth_image, 1);
            

            // Preprocess the point cloud
            // Preprocessing preprocessor(cloud_xyz);
            // preprocessor.voxelize(0.01);
            // preprocessor.removeNaNs();


// #ifdef DEBUG
//             std::cout << "### Preprocessing ###" << std::endl;
//             std::cout << "\tInitial cloud size: " << cloud_xyz->points.size() << std::endl;
//             std::cout << "\tPreprocessed cloud size: " << cloud_xyz->points.size() << std::endl;
// #endif

            // SEGMENTATION
            std::stringstream ss;
            ss.str("");
            ss << depth_image->header.stamp.sec << '.' << depth_image->header.stamp.nanosec;
            this->segmenter.setCloudId(ss.str());
            this->segmenter.setInputCloud(cloud_xyz);
            this->segmenter.segment();
            this->publishPolygonArray();
            
#ifdef VISUALIZE // TODO
            // utils::visualizePlanes(planos);
            // utils::visualizeSegmentation(segmenter.getClusters(), segmenter.getInputCloud());
            // utils::visualizeSegmentationWithNormals(segmenter.getClusters(), segmenter.getInputCloud(), segmenter.getNormals());
            // utils::fullDisplay(cloud_rgb, segmenter.getClusters(), planos);
            
            static StreamViewer viewer; // Static instance of StreamViewer

            viewer.update(cloud_xyz, segmenter.getClusters(), segmenter.getPlanes(), {}); // Assuming map_planes is the same as planos for this example
            viewer.spinOnce();
#endif

#ifdef TIMER
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::cout << "\tTIME: " << duration.count() << " ms" << std::endl;
            std::cout << "\tFPS: " << 1000.0 / duration.count() << std::endl;
#endif
        }

#ifdef DEBUG
        std::cout << "Waiting for camera info..." << std::endl;
#endif
    }


    private:
    PeacSegmenter segmenter;
    sensor_msgs::msg::CameraInfo depth_intrinsics;
    sensor_msgs::msg::Image::SharedPtr depth_image;
    custom_ros2_msgs::msg::PolygonArray polygon_array;
    std::queue<sensor_msgs::msg::Image::SharedPtr> depth_image_queue;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
    rclcpp::Publisher<custom_ros2_msgs::msg::PolygonArray>::SharedPtr publisher;
    std::thread process_thread;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    bool camera_info_received = false;
    YAML::Node params;
};







int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthPlaneSegNode>());
    rclcpp::shutdown();
    return 0;

}