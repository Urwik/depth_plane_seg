#include <iostream>
#include <vector>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/PointIndices.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include "segmentation/boundary_extraction/graph.hpp"

#define DEBUG
#define VISUALIZE

/**
 * @brief Class to represent a plane boundary
 * @note This class is not working, it is just a draft
 */
class PlaneBoundary {

    public:
    PlaneBoundary();
    PlaneBoundary(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& indices, const Eigen::Vector4f& coeffs = Eigen::Vector4f::Zero(), const int& num_edges = 4);
    ~PlaneBoundary();

    void run();


    private:
    void initializeGraphNodes();
    void initializeGraphEdges();
    void getBoundaryPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointIndices::Ptr& indices);
    void computeCoeffs(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& indices, const float& leaf_size=0.01f);
    void updateNodeWeights();

    int num_nodes;
    Eigen::Vector4f coeffs;
    std::vector<Eigen::Vector3f> boundary;
    Graph graph;
};

PlaneBoundary::PlaneBoundary() {
    this->num_nodes = 4;
    this->coeffs.setZero();
    this->boundary.clear();
    this->graph = Graph();
    }



PlaneBoundary::PlaneBoundary(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& indices, const Eigen::Vector4f& coeffs, const int& num_nodes) {
    this->num_nodes = num_nodes;
    this->coeffs = coeffs;

    if (this->coeffs.isZero()) {
        this->computeCoeffs(cloud, indices);
    }

    this->getBoundaryPoints(cloud, indices);
    this->initializeGraphNodes();
    this->initializeGraphEdges();

}

PlaneBoundary::~PlaneBoundary() {
    this->boundary.clear();
    this->graph.~Graph();
}

void
PlaneBoundary::computeCoeffs(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointIndices::Ptr& indices, const float& leaf_size) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_filtered);

    if(leaf_size > 0.0f){
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_filtered);
        vg.setIndices(indices);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*cloud_filtered);
    }

    Eigen::Vector4f centroid;
    // pcl::compute3DCentroid(*cloud, *indices, centroid);
    pcl::compute3DCentroid(*cloud_filtered, centroid);

    Eigen::Matrix3f covariance;
    // pcl::computeCovarianceMatrixNormalized(*cloud, *indices, centroid, covariance);
    pcl::computeCovarianceMatrixNormalized(*cloud_filtered, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance);
    Eigen::Vector3f normal = eigensolver.eigenvectors().col(0);

    this->coeffs << normal[0], normal[1], normal[2], -normal.dot(centroid.head(3));

    if (this->coeffs[3] < 0) {
        this->coeffs = -this->coeffs;
    }
}




void
PlaneBoundary::getBoundaryPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointIndices::Ptr& indices){

    // If no matter of edges, return
    if (this->num_nodes <=0)
        return;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.push_back(this->coeffs[0]);
    coefficients->values.push_back(this->coeffs[1]);
    coefficients->values.push_back(this->coeffs[2]);
    coefficients->values.push_back(this->coeffs[3]);

    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setIndices(indices);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_cloud);

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> hull_indices;
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (projected_cloud);
    chull.reconstruct(*hull_cloud, hull_indices);

    pcl::PointIndices::Ptr boundary_indices(new pcl::PointIndices);
    chull.getHullPointIndices(*boundary_indices);

    // Initialize boundary
    for (size_t i = 0; i<boundary_indices->indices.size(); i++) {
        pcl::PointXYZ p = cloud->points[boundary_indices->indices[i]];
        Eigen::Vector3f point;
        point << p.x, p.y, p.z;
        this->boundary.push_back(point);
    }

#ifdef DEBUG
    std::cout << "### Boundary Points ###" << std::endl;
    std::cout << "\tBoundary size: " << this->boundary.size() << std::endl;
#endif

#ifdef DEBUG && VISUALIZE
    // Visualize the boundary
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Boundary"));

    int v_left = 1;
    int v_right = 2;

    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v_left);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v_right);
    viewer->setBackgroundColor(1.0, 1.0, 1.0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, red, "cloud", v_left);

    for (size_t i = 0; i < this->boundary.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = this->boundary[i].x();
        p.y = this->boundary[i].y();
        p.z = this->boundary[i].z();

        viewer->addText3D(std::to_string(i), p, 100, 0.0, 0.0, 0.0, "text_" + std::to_string(i), v_right);
        viewer->addSphere(p, 10.0, 1.0, 0.0, 0.0, "sphere_" + std::to_string(i), v_right);
    }
    
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }

#endif

}

void
PlaneBoundary::updateNodeWeights() {


    for (size_t i = 0; i < this->graph.nodes.size(); i++) {

        Graph::Node node = this->graph.nodes[i];
        Graph::Edge prev_edge;
        Graph::Edge next_edge;

        // Search for the eges connected to the node
        for (size_t j = 0; j < this->graph.edges.size(); j++) {

            // Look for previous edge
            if (this->graph.edges[j].target == node.idx)
                prev_edge = this->graph.edges[j];

            // Look for next edge
            if (this->graph.edges[j].source == node.idx)
                next_edge = this->graph.edges[j];
        }


        Eigen::Vector3f prev_point = this->boundary[prev_edge.source];
        Eigen::Vector3f current_point = this->boundary[node.idx];
        Eigen::Vector3f next_point = this->boundary[next_edge.target];

        Eigen::Vector3f normal = this->coeffs.head(3);

        Eigen::Vector3f area_vec = (prev_point-current_point).cross(next_point-current_point);
        double area = area_vec.dot(normal) / 2;
        
        this->graph.nodes[i].weight = area + prev_edge.weight + next_edge.weight;
    }
}

void 
PlaneBoundary::initializeGraphNodes() {

    Graph::Node node;

    for (size_t i = 1; i < this->boundary.size(); i++) {
        node.idx = i;
        node.weight = 0.0;
        this->graph.addNode(node);
        this->updateNodeWeight(i);
    }

}

void
PlaneBoundary::initializeGraphEdges() {
    Graph::Edge edge;

    for (size_t i = 1; i < this->boundary.size(); i++) {
        edge.source = i-1;
        edge.target = i;
        edge.weight = 0.0;
        this->graph.addEdge(edge);
    }
}

void
PlaneBoundary::run() {

    while (this->graph.nodesSize() > this->num_nodes) {

        Graph::Node top_node = this->graph.nodes[0];
        this->graph.nodes.erase(this->graph.nodes.begin());

        // Remove the edges connected to the top node and create new edge
        Graph::Edge new_edge;
        new_edge.weight = top_node.weight;
        for (auto it = this->graph.edges.begin(); it != this->graph.edges.end();) {
            
            if (it->source == top_node.idx) {
                new_edge.target = it->target;
                this->graph.edges.erase(it);
                break;
            }
            
            else if (it->target == top_node.idx) {
                new_edge.source = it->source;
                this->graph.edges.erase(it);
                break;
            }
            else
                it++;
        }

        // Add the new edge
        this->graph.edges.push_back(new_edge);


        // Update the weights of the nodes
        this->updateNodeWeights();
    }
}


int main() {

    // Test the PlaneBoundary class

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    float LO_X = -1000.0;
    float HI_X = 1000.0;
    float LO_Y = -500.0;
    float HI_Y = 500.0;
    float LO_Z = -0.01;
    float HI_Z = 0.01;


    Eigen::Vector4f coeffs(1.0, 1.0, 1.0, 1500.0);

    for (size_t i = 0; i < 2000; i++) {

        float x = LO_X + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI_X-LO_X)));
        float y = LO_Y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI_Y-LO_Y)));
        float z_noise = LO_Z + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI_Z-LO_Z)));
        float z = (-coeffs[3] - coeffs[0]*x - coeffs[1]*y) / coeffs[2];
        z += (z*z_noise);

        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = z;
        cloud->push_back(p);
        indices->indices.push_back(i);
    }

    PlaneBoundary boundary(cloud, indices, coeffs, 4);

    return 0;
}