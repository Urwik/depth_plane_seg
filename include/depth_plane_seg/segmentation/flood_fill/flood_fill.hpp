// This file implements a depth-based flood fill algorithm for image segmentation.

#ifndef FLOOD_FILL_HPP
#define FLOOD_FILL_HPP

#include <iostream>
#include <queue>
#include <vector>
#include <numeric>

#include <opencv2/opencv.hpp>


enum NeighbourhoodMask
{
    FOUR = 4,
    EIGHT = 8
};

inline size_t toIndex(const size_t& x, const size_t& y, const size_t& cols)
{
    return x * cols + y;
}

inline std::pair<size_t, size_t> toPixel(const size_t& idx, const size_t& cols)
{
    return std::make_pair(idx / cols, idx % cols);
}

/**
 * @brief Class to perform a depth-based flood fill algorithm
 * @note This class is not working, it is just a draft
 */
class DepthFloodFill
{

public:
    
    DepthFloodFill();
    DepthFloodFill(cv::Mat depth_image);
    ~DepthFloodFill();

    void setDepthImage(cv::Mat depth_image);
    cv::Mat getDepthImage() const { return depth_image; }

    void
    run(const std::pair<size_t, size_t>& seed = std::pair<size_t, size_t>(0, 0), NeighbourhoodMask type = NeighbourhoodMask::EIGHT);
    cv::Mat getClusteredImage() const { return clust_image; }

    private:
    cv::Mat depth_image;
    cv::Mat clust_image;
};

DepthFloodFill::DepthFloodFill() 
{
    depth_image.setTo(0);
    clust_image.setTo(0);
}

DepthFloodFill::DepthFloodFill(cv::Mat depth_image)
{
    this->depth_image = depth_image;
    this->clust_image = cv::Mat::zeros(depth_image.size(), CV_8UC1);
}

DepthFloodFill::~DepthFloodFill()
{
    this->depth_image.release();
    this->clust_image.release();
}

void DepthFloodFill::setDepthImage(cv::Mat depth_image)
{
    this->depth_image = depth_image;
    this->clust_image = cv::Mat::zeros(depth_image.size(), CV_8UC1);
}

void
DepthFloodFill::run(const std::pair<size_t, size_t>& seed, NeighbourhoodMask type)
{
    std::vector<std::vector<size_t>> clusters;
    std::set<size_t> visited;
    clust_image = cv::Mat::zeros(depth_image.size(), CV_8UC1);

    int rows = depth_image.rows;
    int cols = depth_image.cols;
    
    // SET MASK OF THE FLOOD FILL
    std::vector<std::pair<int, int>> directions;
    switch (type) {
    
    case NeighbourhoodMask::FOUR:
          // Direcciones para 8 vecinos
        directions = {
        {0, 1},  // Derecha
        {1, 0},  // Abajo
        {0, -1}, // Izquierda
        {-1, 0} // Arriba;
        };
        break;
    
    case NeighbourhoodMask::EIGHT:
        // Direcciones para 8 vecinos
        directions = {
            {0, 1},  // Derecha
            {1, 1},  // Abajo Derecha
            {1, 0},  // Abajo
            {1, -1}, // Abajo Izquierda
            {0, -1}, // Izquierda
            {-1, -1}, // Arriba Izquierda
            {-1, 0}, // Arriba
            {-1, 1} // Arriba Derecha
        };
        break;

    default:
        directions = {
            {0, 1},  // Derecha
            {1, 1},  // Abajo Derecha
            {1, 0},  // Abajo
            {1, -1}, // Abajo Izquierda
            {0, -1}, // Izquierda
            {-1, -1}, // Arriba Izquierda
            {-1, 0}, // Arriba
            {-1, 1} // Arriba Derecha
        };
        break;
    }

    size_t cluster_id_count = 0;


    // NEW SEED
    while (visited.size() < (rows * cols)*0.9) {
    
        // NEW CLUSTER
        clusters.push_back(std::vector<size_t>());
        
        // FIND NEW SEED
        size_t seed_idx = 0;
        while (true){
            seed_idx = static_cast<size_t>(std::rand() % (rows * cols));
            if (visited.find(seed_idx) == visited.end())
                break;
        }
    
        std::queue<std::pair<size_t, size_t>> queue;
        queue.push(toPixel(seed_idx, cols));

        // EXPAND CURREND SEED
        while (!queue.empty())
        {
            // Get the first element of the queue
            std::pair<size_t, size_t> curr_pixel = queue.front();

            // Remove the first element of the queue
            queue.pop();

            // Get the coordinates of the element
            int x = curr_pixel.first;
            int y = curr_pixel.second;

            // DISCARD PIXELS 
            if (x < 0 || x >= rows || y < 0 || y >= cols) // TODO: NOT SURE IF THIS IS NECESSARY
                continue;
            if (depth_image.at<float>(x, y) <= 0) // TODO: NOT SURE IF THIS IS NECESSARY
                continue;
            
            // EXAMINE NEIGHBOURS
            for (auto &direction : directions)
            {
                int nx = x + direction.first;
                int ny = y + direction.second;

                // DISCARD PIXELS OUT OF BOUNDS
                if (nx < 0 || nx >= rows || ny < 0 || ny >= cols)
                    continue;

                // DISCARD PIXELS WITH DEPTH 0
                if (depth_image.at<float>(nx, ny) <= 0)
                    continue;

                // DISCARD PIXELS ALREADY ASSIGNED
                size_t pixel_idx = toIndex(nx, ny, cols);
                for( const auto& cluster : clusters)
                {
                    if (std::find(cluster.begin(), cluster.end(), pixel_idx) != cluster.end())
                        continue;
                }

                // DISCARD PIXELS BY DEPTH DIFFERENCE (EDGE PIXELS)
                if (std::fabs(depth_image.at<float>(nx, ny) - depth_image.at<float>(x, y)) > 0.05)
                    continue;

                // ADD PIXEL TO THE QUEUE
                queue.push(std::make_pair(nx, ny));
                visited.insert(pixel_idx);
                clusters[cluster_id_count].push_back(pixel_idx);
                clust_image.at<uchar>(nx, ny) = (255 * cluster_id_count + 1) % 256;
            }
        } // end queue
        cluster_id_count++;
    } // all visited
}
#endif // FLOOD_FILL_HPP