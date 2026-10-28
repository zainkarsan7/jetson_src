#pragma once
#include <cstddef>
#include <limits>
#include <vector>

namespace hb_robot_skills::motion{


using NodeId = std::size_t;
using EdgeId = std::size_t;
constexpr NodeId INVALID_NODE = std::numeric_limits<NodeId>::max();


enum class TransitStatus{
    Unknown,
    Valid,
    FailedWithBudget,
    Invalid
};

struct GraphNode{
    NodeId id;
    std::size_t layer;
    double process_cost = 0.0;

};


struct GraphEdge{
    EdgeId id;
    NodeId from;
    NodeId to;
    
    double estimated_cost = 0.0;
    double actual_cost = 0.0;
    TransitStatus status = TransitStatus::Unknown;
};

struct GraphRoute{
    std::vector<NodeId> nodes;
    std::vector<EdgeId> edges;

    double cost = std::numeric_limits<double>::infinity();
    bool found() const{
        return !nodes.empty();
    }

};

class CutCandidateGraph{
    public:

        NodeId addNode(
            std::size_t layer, double process_cost
        );

        EdgeId addEdge(
            NodeId from, NodeId to, double estimated_cost
        );

        GraphRoute cheapestRoute(NodeId start,NodeId goal, double unknown_edge_penalty=0.0,double failed_edge_penalty=1000.0)const;
        GraphEdge& edge(EdgeId id);
        const GraphEdge& edge(EdgeId id)const;
        const GraphNode& node(NodeId id)const;
        const std::vector<GraphEdge>&edges() const;



    private:

        std::vector<GraphNode> nodes_;
        std::vector<GraphEdge> edges_;
};


}