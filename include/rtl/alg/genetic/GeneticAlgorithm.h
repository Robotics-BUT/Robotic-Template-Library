// This file is part of the Robotic Template Library (RTL), a C++
// template library for usage in robotic research and applications
// under the MIT licence:
//
// Copyright 2021 Brno University of Technology
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom
// the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
// OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Contact person: Adam Ligocki <adam.ligocki@vutbr.cz>

#ifndef ROBOTICTEMPLATELIBRARY_GENETICALGORITHM_H
#define ROBOTICTEMPLATELIBRARY_GENETICALGORITHM_H

namespace rtl
{
    template<typename AgentType, size_t agents_in_epoch, size_t surviving_elites, size_t surviving_total, size_t mutations_per_epoch>
    class GeneticAlgorithm {

        static_assert(agents_in_epoch > surviving_total);
        static_assert(surviving_elites < surviving_total);

    public:

        GeneticAlgorithm() {
            init();
        }

        void iterate_epoch() {
            next_epoch_agents_.clear();
            next_epoch_agents_.reserve(agents_in_epoch);

            agents_evaluation();
            selection();
            reproduction();
            mutation();

            agents_ = next_epoch_agents_;
        }

        AgentType best_agent(size_t n = 0) {
            agents_evaluation();
            sort_agents();
            return agents_.at(n).first;
        }

    private:

        void init() {

            std::random_device r;
            engine_ = std::default_random_engine(r());
            distribution_ = std::uniform_real_distribution<float>(0, 1);

            agents_.reserve(agents_in_epoch);
            for (size_t i = 0 ; i < agents_in_epoch ; i++) {
                agents_.push_back({AgentType::random(), 0.0f});
            }
        }

        void agents_evaluation() {
            float cum_sum = 0.0f;

            for(auto& agent : agents_) {
                float score = agent.first.score();
                agent.second = score;
                cum_sum += score;
            }

            for(auto& agent : agents_) {agent.second /= cum_sum;}
        }

        void selection() {
            select_elites();
            select_random();
        }

        void select_elites() {
            sort_agents();
            for (size_t i = 0 ; i < surviving_elites ; i+=1) {
                next_epoch_agents_.push_back(agents_.at(i));
            }
        }

        void sort_agents() {
            sort(agents_.begin(), agents_.end(), [](const std::pair<AgentType, float>& a, const std::pair<AgentType, float>& b) -> bool {
                return a.second > b.second;
            });
        }

        void select_random() {
            for (size_t i = next_epoch_agents_.size() ; i < surviving_total ; i+=1) {
                next_epoch_agents_.push_back(agents_.at(get_random_index(agents_.size())));
            }
        }

        void mutation() {
            for (size_t i = 0 ; i < mutations_per_epoch ; i+=1) {
                // do not mutate the best agent
                next_epoch_agents_.at(get_random_index(next_epoch_agents_.size()-1)+1).first.mutate();
            }
        }

        void reproduction() {
            for (size_t i = next_epoch_agents_.size(); i < agents_in_epoch; i += 1) {
                auto rand_index_1 = get_random_index(agents_.size());
                auto rand_index_2 = get_random_index(agents_.size());
                next_epoch_agents_.push_back({agents_.at(rand_index_1).first.crossover(agents_.at(rand_index_2).first), 0.0f});
            }
        }

        size_t get_random_index(size_t range) {
            return static_cast<size_t>(distribution_(engine_) * static_cast<float>(range-1));
        }

        std::vector<std::pair<AgentType, float>> agents_;
        std::vector<std::pair<AgentType, float>> next_epoch_agents_;

        std::default_random_engine engine_;
        std::uniform_real_distribution<float> distribution_;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_GENETICALGORITHM_H
