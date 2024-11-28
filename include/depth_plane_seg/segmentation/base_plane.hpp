#ifndef BASE_PLANE_HPP
#define BASE_PLANE_HPP

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>

/**
 * @brief Class to represent a plane in 3D space
 */
class Plane {
    public:
        Plane();
        Plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& indices, const Eigen::Vector4f& in_coeffs = Eigen::Vector4f::Zero());
        ~Plane();
        
        void setCoeffs(Eigen::Vector4f coeffs);

        void computeCoeffs(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& indices);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr
        projectIntoPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices);
        
        void extractEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud);

        Eigen::Vector4f coeffs;
        Eigen::Vector3f center;
        std::vector<Eigen::Vector3f> edges;

#ifdef TIMER
        double computeCoeffs_time =0.0, extractEdgePoints_time=0.0, projectIntoPlane_time=0.0 ;
#endif
};

/**
 * @brief Construct a new Plane object
 */
Plane::Plane() {
    this->coeffs.setZero();
    this->edges.clear();
}

/**
 * @brief Construct a new Plane object
 * @param cloud Point cloud
 * @param indices Indices of the points to consider
 * @param in_coeffs Coefficients of the plane
 */
Plane::Plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& indices, const Eigen::Vector4f & in_coeffs) {

    this->coeffs = in_coeffs;
    this->edges.clear();

    if (this->coeffs.isZero()){
#ifdef TIMER
        Timer timer;
        timer.start();
#endif
        this->computeCoeffs(cloud, indices);

#ifdef TIMER
        timer.end();
        std::cout << "Plane computeCoeffs time: " << timer.currentElapsedTime() << " ms" << std::endl;
#endif
    }

    this->extractEdgePoints( this->projectIntoPlane(cloud, indices) );
}

/**
 * @brief Destroy the Plane object
 */
Plane::~Plane() {
    this->edges.clear();
}

/**
 * @brief Set the coefficients of the plane
 * @param coeffs Coefficients of the plane
 */
void 
Plane::setCoeffs(Eigen::Vector4f coeffs) {
    this->coeffs = coeffs;
}


/**
 * @brief Compute the coefficients of the plane using the SVD on the covariance matrix
 * @param cloud Point cloud
 * @param indices Indices of the points to consider
 */
void
Plane::computeCoeffs(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointIndices::Ptr& indices) {
#ifdef DEBUG_PLANE
    std::cout << "computingCoeffs..." << std::endl;
#endif

#ifdef TIMER
    Timer timer;
    timer.start();
#endif

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_filtered);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setIndices(indices);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);

    Eigen::Vector4f centroid;
    // pcl::compute3DCentroid(*cloud, *indices, centroid);
    pcl::compute3DCentroid(*cloud_filtered, centroid);

    Eigen::Matrix3f covariance;
    // pcl::computeCovarianceMatrixNormalized(*cloud, *indices, centroid, covariance);
    pcl::computeCovarianceMatrixNormalized(*cloud_filtered, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance);
    Eigen::Vector3f normal = eigensolver.eigenvectors().col(0);

    this->coeffs << normal[0], normal[1], normal[2], -normal.dot(centroid.head(3));

#ifdef TIMER
    this->computeCoeffs_time = timer.currentElapsedTime();
#endif

#ifdef DEBUG_PLANE
    std::cout << "\tcoeffs: " << this->coeffs.transpose() << std::endl;
#endif
}

/* void
Plane::computeCoeffs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices) {

    #ifdef DEBUG
    std::cout << "computingCoeffs..." << std::endl;
    #endif

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMaxIterations(1000);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud);
    seg.setIndices(indices);
    seg.segment(*inliers, *coefficients);

    this->coeffs << coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3];

    #ifdef DEBUG
    std::cout << "\tcoeffs: " << this->coeffs.transpose() << std::endl;
    #endif

} */


/**
 * @brief Project the points into the plane
 * @param cloud Point cloud
 * @param indices Indices of the points to consider
 * @return Projected point cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr 
Plane::projectIntoPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices) {

#ifdef DEBUG_PLANE
    std::cout << "- Projecting into plane..." << std::endl;
#endif

#ifdef TIMER
    Timer timer;
    timer.start();
#endif

    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    

    coefficients->values.push_back(this->coeffs[0]);
    coefficients->values.push_back(this->coeffs[1]);
    coefficients->values.push_back(this->coeffs[2]);
    coefficients->values.push_back(this->coeffs[3]);

#ifdef DEBUG_PLANE
    std::cout << "coeffs: " << this->coeffs.transpose() << std::endl;
    std::cout << "coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
#endif 

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setIndices(indices);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_cloud);

#ifdef TIMER
    this->projectIntoPlane_time = timer.currentElapsedTime();
#endif

    return projected_cloud;
}


/**
 * @brief Extract the edge points of the plane
 * @param projected_cloud Projected point cloud
 */
void
Plane::extractEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud) {

#ifdef DEBUG_PLANE
    std::cout << "running extractEdgePoints..." << std::endl;
#endif

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*projected_cloud, *projected_cloud, *indices);

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (projected_cloud);
    chull.reconstruct (*cloud_hull);

    // Compute the centroid of the convex hull
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_hull, centroid);
    this->center << centroid[0], centroid[1], centroid[2];

    for(const pcl::PointXYZ &point : cloud_hull->points) {
        Eigen::Vector3f edge;
        edge << point.x, point.y, point.z;
        this->edges.push_back(edge);
    }
}

#endif // BASE_PLANE_HPP