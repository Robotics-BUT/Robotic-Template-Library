//
// Created by default_user on 10/3/22.
//

#ifndef ROBOTICTEMPLATELIBRARY_PATHFINDERNODE_H
#define ROBOTICTEMPLATELIBRARY_PATHFINDERNODE_H

#include <vector>
#include <array>

namespace rtl {

    template<typename costDType = float>
    class PathFinderNode {
    public:
        [[nodiscard]] costDType getForwardCost() const { return forward_cost_; }
        [[nodiscard]] costDType getHeuristicCost() const { return heuristic_cost_; }
        [[nodiscard]] costDType getCombinedCost() const { return forward_cost_ + heuristic_cost_; }

        void setForwardCost(costDType g_cost) { forward_cost_ = g_cost; }
        void setHeuristicCost(costDType h_cost) { heuristic_cost_ = h_cost; }

        void addNeighbour(PathFinderNode& node) {
            neighbours_.template emplace_back(node);
        }

        const std::vector<std::reference_wrapper<PathFinderNode>>& getNeighbours() {return neighbours_;}

    private:
        costDType forward_cost_;
        costDType heuristic_cost_;
        std::vector<std::reference_wrapper<PathFinderNode>> neighbours_ = {};
    };
}

#endif //ROBOTICTEMPLATELIBRARY_PATHFINDERNODE_H
