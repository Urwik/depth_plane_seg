#ifndef PEAC_INTERFACE_HPP
#define PEAC_INTERFACE_HPP

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "base_segmenter.hpp"
#include "peac/include/AHCPlaneFitter.hpp"

class PeacSegmenter : public BaseSegmenter {

    public:
        PeacSegmenter();
        ~PeacSegmenter();

        void saveSegImage(const cv::Mat& seg_image);
        void segment() override;
        std::vector<Plane> getPlanes() override;
        void loadParams();

    private:
        void segImg2Clusters(cv::Mat seg);
        void extractCluster(std::vector<std::vector<int>> &seg_clusters);
        void computePlaneEdgesPeac(const std::vector<ahc::PlaneSeg::shared_ptr> &extracted_planes);
        ahc::PlaneFitter<ahc::OrganizedImage3D<pcl::PointXYZ>> pf;
};


PeacSegmenter::PeacSegmenter() {
    this->input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

PeacSegmenter::~PeacSegmenter() {
    this->input_cloud.reset();
}


/**
 * @brief Load the parameters from the YAML file
 * @return void
 * @note Params inherited from BaseSegmenter class using the setParams function
 */
void 
PeacSegmenter::loadParams() {


    // Apply scaling factor to the input cloud
    float unitScaleFactor = this->params["unitScaleFactor"].as<float>();
    pcl::transformPointCloud<pcl::PointXYZ>(*this->input_cloud, *this->input_cloud, Eigen::Affine3f(Eigen::UniformScaling<float>((float)unitScaleFactor)));

#ifdef DEBUG_INIT
    this->pf.saveDir = this->params["outputDir"].as<std::string>();
#endif

   	this->pf.minSupport = this->params["minSupport"].as<int>();
	this->pf.windowWidth = this->params["windowWidth"].as<int>();
	this->pf.windowHeight = this->params["windowHeight"].as<int>();
	this->pf.doRefine = this->params["doRefine"].as<bool>();

	this->pf.params.initType = (ahc::InitType) this->params["initType"].as<int>();

	//T_mse
	this->pf.params.stdTol_merge = this->params["stdTol_merge"].as<double>();
	this->pf.params.stdTol_init = this->params["stdTol_init"].as<double>();
	this->pf.params.depthSigma = this->params["depthSigma"].as<double>();

	//T_dz
	this->pf.params.depthAlpha = this->params["depthAlpha"].as<double>();
	this->pf.params.depthChangeTol = this->params["depthChangeTol"].as<double>();

	//T_ang
	this->pf.params.z_near = this->params["z_near"].as<double>();
	this->pf.params.z_far = this->params["z_far"].as<double>();
	this->pf.params.angle_near = MACRO_DEG2RAD( this->params["angleDegree_near"].as<double>() );
	this->pf.params.angle_far = MACRO_DEG2RAD( this->params["angleDegree_far"].as<double>() );
	this->pf.params.similarityTh_merge = std::cos(MACRO_DEG2RAD(this->params["similarityDegreeTh_merge"].as<double>()));
	this->pf.params.similarityTh_refine = std::cos(MACRO_DEG2RAD(this->params["similarityDegreeTh_refine"].as<double>()));

    this->pf.params.nanTh = this->params["nanTh"].as<float>();

    #ifdef DEBUG
    std::cout << "Loaded parameters:" << std::endl;
    #ifdef DEBUG_INIT
    std::cout << "\toutputDir: " << this->pf.saveDir << std::endl;
    #endif
    std::cout << "\tunitScaleFactor: " << unitScaleFactor << std::endl;
    std::cout << "\tnanTh: " << this->pf.params.nanTh << std::endl;
    std::cout << "\tstdTol_merge: " << this->pf.params.stdTol_merge << std::endl;
    std::cout << "\tstdTol_init: " << this->pf.params.stdTol_init << std::endl;
    std::cout << "\tdepthSigma: " << this->pf.params.depthSigma << std::endl;
    std::cout << "\tz_near: " << this->pf.params.z_near << std::endl;
    std::cout << "\tz_far: " << this->pf.params.z_far << std::endl;
    std::cout << "\tangleDegree_near (radians): " << this->pf.params.angle_near << std::endl;
    std::cout << "\tangleDegree_far (radians): " << this->pf.params.angle_far << std::endl;
    std::cout << "\tsimilarityDegreeTh_merge (radians): " << this->pf.params.similarityTh_merge << std::endl;
    std::cout << "\tsimilarityDegreeTh_refine (radians): " << this->pf.params.similarityTh_refine << std::endl;
    std::cout << "\tdepthAlpha: " << this->pf.params.depthAlpha << std::endl;
    std::cout << "\tdepthChangeTol: " << this->pf.params.depthChangeTol << std::endl;
    std::cout << "\tinitType: " << this->pf.params.initType << std::endl;
    std::cout << "\tminSupport: " << this->pf.minSupport << std::endl;
    std::cout << "\twindowWidth: " << this->pf.windowWidth << std::endl;
    std::cout << "\twindowHeight: " << this->pf.windowHeight << std::endl;
    std::cout << "\tdoRefine: " << this->pf.doRefine << std::endl;
    #endif



}


/**
 * @brief Convert the segmented image to vector of PointIndices of the input cloud
 * @param seg Segmented image
 * @return void
 * @note Clusters are inherited from BaseSegmenter class
 */
void
PeacSegmenter::segImg2Clusters(cv::Mat seg) {
    // Ensure extractedPlanes is not empty
    if (this->pf.extractedPlanes.empty()) {
        std::cerr << "Error: extractedPlanes is empty." << std::endl;
        return;
    }

    this->clusters.clear();
    this->clusters.resize(this->pf.extractedPlanes.size());
    std::vector<cv::Vec3b> colors;
    size_t cluster_id_cnt = 0;
    std::map<size_t, cv::Vec3b> cluster_colors;

    for (int y = 0; y < seg.rows; ++y) {
        for (int x = 0; x < seg.cols; ++x) {
            cv::Vec3b color = seg.at<cv::Vec3b>(y, x);

            // Add color to colors vector if it is not already there
            if (std::find(colors.begin(), colors.end(), color) == colors.end()) {
                cluster_colors[cluster_id_cnt] = color;
                colors.push_back(color);
                cluster_id_cnt++;
            }

            // Find the cluster id using the map
            auto it = std::find_if(cluster_colors.begin(), cluster_colors.end(),
                                   [&color](const std::pair<size_t, cv::Vec3b>& keyval) {
                                       return keyval.second == color;
                                   });

            if (it != cluster_colors.end()) {
                size_t cluster_id = it->first;
                this->clusters[cluster_id].indices.push_back(y * seg.cols + x);
            }
        }
    }
}


/**
 * @brief Converts the output of PEAC segmenter to common segmentation output
 * std::vector<pcl::PointIndices> clusters
 * @param seg_clusters Segmented clusters from PEAC
 * @return void
 * @note Clusters are inherited from BaseSegmenter class
 */
void
PeacSegmenter::extractCluster(std::vector<std::vector<int>> &seg_clusters) {

#ifdef DEBUG
    std::cout << "Converting Peac output to vector<pcl::PointIndices>..." << std::endl;
#endif

    this->clusters.clear();
    this->clusters.resize(seg_clusters.size());

    // #pragma omp parallel for
    for (size_t i = 0; i < seg_clusters.size(); ++i) {
        for (size_t j = 0; j < seg_clusters[i].size(); ++j) {
            this->clusters[i].indices.push_back(seg_clusters[i][j]);
        }
    }

// #ifdef DEBUG
//     int tmp_count = 0;
//     int total_points = 0;
//     for (const pcl::PointIndices & plane : this->clusters)
//     {
//         std::cout << "\t#" << tmp_count << " size: " << plane.indices.size() << std::endl;
//         total_points += plane.indices.size();
//         tmp_count++;
//     }

//     std::cout << "Num of segmented points: " << total_points << std::endl;
// #endif
}


/**
 * @brief Compute the plane edges from the extracted planes
 * @param extracted_planes Extracted planes from PEAC
 * @return void
 * @note Planes are inherited from BaseSegmenter class
 */
void
PeacSegmenter::computePlaneEdgesPeac(const std::vector<ahc::PlaneSeg::shared_ptr> &extracted_planes){

#ifdef DEBUG
    std::cout << " - Extracting Plane Edges..." << std::endl;
#endif

    pcl::PointIndices::Ptr plane_indices (new pcl::PointIndices);
    

    for (size_t i = 0; i < this->clusters.size(); i++) {
        plane_indices->indices.clear();
        *plane_indices = this->clusters[i];
    
        std::vector<Eigen::Vector4f> coeffs;
        for(auto &plane : extracted_planes) {
            Eigen::Vector4f coeffs_tmp;
            Eigen::Vector3f center(plane->center[0], plane->center[1], plane->center[2]);
            Eigen::Vector3f normal(plane->normal[0], plane->normal[1], plane->normal[2]);
            double d = -center.dot(normal);

            coeffs_tmp << normal(0), normal(1), normal(2), d;
            coeffs.push_back(coeffs_tmp);
        }

        Plane plane_model(this->input_cloud, plane_indices, coeffs[i]);    

#ifdef DEBUG_PLANE
        std::cout << "\tplane_model.center" << plane_model.center << std::endl;
        std::cout << "\tplane_model.coeffs" << plane_model.coeffs.transpose() << std::endl;

        for (const Eigen::Vector3f &edge : plane_model.edges) {
            std::cout << "\t\tedge: " << edge.transpose() << std::endl;
        }

#endif
        this->planes.push_back(plane_model);
    }

}


/**
 * @brief Main function of the class, segment the input cloud using PEAC
 */
void
PeacSegmenter::segment(){
    
    std::vector<std::vector<int>> seg_clusters;
	cv::Mat seg_image(this->input_cloud->height, this->input_cloud->width, CV_8UC3);

    #ifdef DEBUG
    std::cout << "# Segmenting using PEAC..." << std::endl;
    #endif

    ahc::OrganizedImage3D<pcl::PointXYZ> Ixyz(*this->input_cloud);

    // this->loadParams();
	
    // RUN PEAC
    this->planes.clear();
    this->clusters.clear();
    this->pf.run(&Ixyz, &seg_clusters, &seg_image);

    // TODO: tmp_clusters do not contain non clustered points
    this->extractCluster(seg_clusters);
    this->computePlaneEdgesPeac(this->pf.extractedPlanes); 
    // this->segImg2Clusters(seg);
    
    
    if(this->params["saveSegCloud"].as<bool>()){
        this->saveSegCloud();
    }

    if(this->params["saveSegImage"].as<bool>()){
        this->saveSegImage(seg_image);
    }

	if (this->params["showWindow"].as<bool>())
    {
        cv::namedWindow("seg");
		cv::imshow("seg", seg_image);
		cv::waitKey(10);
	}

}


/**
 * @brief Save the segmented image to the output directory
 * @param seg_image Segmented image
 * @return void
 */
void
PeacSegmenter::saveSegImage(const cv::Mat& seg_image){
        cv::cvtColor(seg_image, seg_image, cv::COLOR_RGB2BGR);

        fs::path output_dir = this->params["outputDir"].as<std::string>();
        std::string cloud_name = fs::path(this->cloud_id).stem().string();

        std::string out_path = output_dir.string() + "/" + cloud_name + ".png";

    	cv::imwrite(out_path, seg_image);
}


/**
 * @brief Get the planes from the PEAC segmenter
 * @return std::vector<Plane> Planes
 */
std::vector<Plane>
PeacSegmenter::getPlanes(){

    // if (this->planes.empty()) {
    //     std::cerr << "Error: Planes are empty." << std::endl;
    //     this->computePlaneEdgesPeac(this->pf.extractedPlanes);
    // }
    return this->planes;

}

#endif // PEAC_INTERFACE_HPP