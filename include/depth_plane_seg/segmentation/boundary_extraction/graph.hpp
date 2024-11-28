#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <set>
#include <vector>
#include <queue>
#include <algorithm>
#include <Eigen/Core>

/**
 * @brief Class to represent a graph
 * @note This class is not working, it is just a draft
 */
class Graph {

    public:
    struct Node {
        size_t idx;
        Eigen::Vector3f point;
        double weight;

        Node() {
            this->idx = 0;
            this->point = Eigen::Vector3f::Zero();
            this->weight = 0.0;
        }
        
        bool operator()(const Node &a, const Node &b) const
        {
            return a.weight > b.weight;
        }
    };

    struct Edge {
        size_t source;
        size_t target;
        double weight;

        Edge() {
            this->source = 0;
            this->target = 0;
            this->weight = 0.0;
        }
    };

    
    Graph();
    ~Graph();

    int nodesSize() { return this->nodes.size(); }

    void addNode(const Node& node);
    void addNode(const size_t& idx, const double& weight);
    void removeTopNode();
    void removeNode(const size_t& idx = 0);
    void updateNodeWeight(const size_t& idx, const double& weight);
    void updateNodesWeights();
    double computeWeight(const size_t& idx);


    void addEdge(const Edge& edge);
    void removeEdge(const Edge& edge);
    void removeEdge(const size_t& source, const size_t& target);
    void updateEdgeWeight(const size_t& idx, const double& weight);
    Edge getEdge(const size_t& idx);
    std::vector<Edge> getEdges() { return this->edges; }
    

    std::vector<Node> nodes;
    std::vector<Edge> edges;

    private:
    void sortNodes();
};

Graph::Graph() {
    this->nodes.empty();
    this->edges.clear();
} 

Graph::~Graph() {
    this->nodes.empty();
    this->edges.clear();
}



// NODE FUNCTIONS
void
Graph::addNode(const Node& node) {
    this->nodes.push(node);
}

void
Graph::addNode(const size_t& idx, const double& weight) {
    Node node;
    node.idx = idx;
    node.weight = weight;
    this->nodes.push(node);
}



void
Graph::removeTopNode() {
    this->nodes.erase(this->nodes.begin());
}

void 
Graph::removeNode(const size_t& idx) {

    for (auto it = this->nodes.begin(); it != this->nodes.end(); it++) {
        if (it->idx == idx) {
            this->nodes.erase(it);
            break;
        }
    }
}

void
Graph::updateNodeWeight(const size_t& idx, const double& weight) {

    // Update the weight of the specified node
    for (size_t i = 0; i < this->nodes.size(); i++) {
        if (this->nodes[i].idx == idx) {
            this->nodes[i].weight = weight;
            break;
        }
    }
}


// double
// Graph::computeWeight(const size_t& idx) {
//     Eigen::Vector3f current_p = this->boundary[idx];
//     Eigen::Vector3f next_p = this->boundary[idx+1];
//     Eigen::Vector3f prev_p = this->boundary[idx-1];

//     Eigen::Vector3f normal = this->coeffs.head(3);

//     Eigen::Vector3f area_vec = (prev_p-current_p).cross(next_p-current_p);
//     double area = area_vec.dot(normal) / 2;

//     Graph::Edge prev_edge = this->edges[idx-1];
//     Graph::Edge next_edge = this->edges[idx];


//     double weight = area + prev_edge.weight + next_edge.weight;
// }


void 
Graph::updateNodesWeights() {
    
    for (size_t i = 0; i < this->nodes.size(); i++) {
        this->nodes[i].weight = this->computeWeight(i);
    }
}



// EDGE FUNCTIONS
void
Graph::addEdge(const Edge& edge) {
    this->edges.push_back(edge);
}

void
Graph::removeEdge(const Edge& edge) {
    for (auto it = this->edges.begin(); it != this->edges.end(); it++) {
        if (it->source == edge.source && it->target == edge.target) {
            this->edges.erase(it);
            break;
        }
    }
}

void
Graph::removeEdge(const size_t& source, const size_t& target) {
    for (auto it = this->edges.begin(); it != this->edges.end(); it++) {
        if (it->source == source && it->target == target) {
            this->edges.erase(it);
            break;
        }
    }
}

void
Graph::updateEdgeWeight(const size_t& idx, const double& weight) {
    this->edges[idx].weight = weight;
}

Graph::Edge
Graph::getEdge(const size_t& idx) {
    return this->edges[idx];
}


void 
Graph::sortNodes() {
    std::sort(this->nodes.begin(), this->nodes.end(), Node());
}

#endif