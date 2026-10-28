#include "hb_robot_skills/motion/cut_candidate_graph.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace hb_robot_skills::motion{


    NodeId CutCandidateGraph::addNode(
            std::size_t layer, double process_cost
        ){
            if(!std::isfinite(process_cost) || process_cost<0.0){
                throw std::invalid_argument("process cost is wrong");
            }

            const NodeId id = nodes_.size();
            nodes_.push_back({id,layer,process_cost});
            return id;
        }

    EdgeId CutCandidateGraph::addEdge(
        NodeId from, NodeId to, double estimated_cost
    ){
        if(!std::isfinite(estimated_cost) || estimated_cost<0.0){
            throw std::invalid_argument("estimated cost is wrong");
        }
        
        if(from >=nodes_.size() || to>= nodes_.size()){
            throw std::invalid_argument("nodes not in graph");
        }
        if(nodes_[from].layer >= nodes_[to].layer){
                throw std::invalid_argument("layer direction is wrong");
            }
        const EdgeId id = edges_.size();
        edges_.push_back({id,from,to,estimated_cost, 0.0,TransitStatus::Unknown});
        return id;
    }

    GraphRoute CutCandidateGraph::cheapestRoute(NodeId start,NodeId goal, double unknown_edge_penalty,double failed_edge_penalty)const{
        if(start >=nodes_.size() || goal>= nodes_.size()){
            throw std::invalid_argument("nodes not in graph");
        }

        const double INF = std::numeric_limits<double>::infinity();
        std::vector<double> cost(nodes_.size(),INF);

        std::vector<EdgeId> predecessor(
            nodes_.size(),edges_.size()
        );

        std::vector<NodeId> order(nodes_.size());
        std::iota(order.begin(),order.end(),0);

        std::stable_sort(order.begin(),order.end(),
        [&](NodeId a, NodeId b){
            return nodes_[a].layer<nodes_[b].layer;
        }
    );
        // set the start cost to whatever cost the node has.
        cost[start] = nodes_[start].process_cost;
        for (NodeId u: order){
            if(!std::isfinite(u)){
                continue;
            }
            for (const GraphEdge & e: edges_){
                // look for all the edges of u
                if(e.from != u){
                    continue;
                }
                if(e.status == TransitStatus::Invalid){
                    continue;
                }
                double edge_cost = e.estimated_cost;
                switch (e.status){
                    case TransitStatus::Valid:
                        edge_cost = e.actual_cost;
                        break;
                    case TransitStatus::Unknown:
                        edge_cost += unknown_edge_penalty;
                        break;
                    case TransitStatus::FailedWithBudget:
                        edge_cost += failed_edge_penalty;
                        break;
                    case TransitStatus::Invalid:
                        continue;
                }
                const double candidate_cost = cost[u] + edge_cost + 
                nodes_[e.to].process_cost;
                // if cheaper route, update the wire from u to e.to
                if(candidate_cost<cost[e.to]){
                    cost[e.to] = candidate_cost;
                    predecessor[e.to] = e.id;
                }
            }
        }

        GraphRoute route;

        if(!std::isfinite(cost[goal]))
            return route;

        route.cost = cost[goal];
        NodeId current = goal;
        route.nodes.push_back(current);

        while(current!= start){
            const EdgeId edge_id = predecessor[current];
            if(edge_id == edges_.size()){
                return{};
            }
            const GraphEdge& e = edges_[edge_id];
            route.edges.push_back(edge_id);
            current = e.from;
            route.nodes.push_back(current);
        }
        std::reverse(route.nodes.begin(),route.nodes.end());
        std::reverse(route.edges.begin(),route.edges.end());

        return route;



    }

    GraphEdge& CutCandidateGraph::edge(EdgeId id){
        return edges_.at(id);
    }

    const GraphEdge& CutCandidateGraph::edge(EdgeId id)const{
        return edges_.at(id);
    }
    const GraphNode& CutCandidateGraph::node(NodeId id)const{
        return nodes_.at(id);
    }


    const std::vector<GraphEdge>& CutCandidateGraph::edges() const{
        return edges_;
    }




}