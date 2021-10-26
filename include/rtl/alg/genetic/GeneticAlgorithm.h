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
    //! Genetic Algorithm Implementation
    /*!
     * Generic implementation of GA (Genetic Algorithm) for a generic agent.
     * The GA implements following phases:
     * 1] Init population
     * 2] Evaluate agenst (calculate score)
     * 3] Select N elites to next epoch
     * 4] Select M agents to survive (higher score, higher chance to survive)
     * 5] Reproduction crossing over survived agents
     * 6] Mutation. Select P agents and mutate them
     * 7] get results and go back to 2
     *
     * The GA is specified by following arguments:
     *
     * @tparam AgentType data type of agent
     * @tparam agents_in_epoch Number of agents at the beginning of each epoch
     * @tparam surviving_elites Number of best agents that survive epoch
     * @tparam surviving_total Number of agents that survives epoch (elites + randomly selected)
     * @tparam mutations_per_epoch Number of mutatons in epoch
     */
    template<typename AgentType, size_t agents_in_epoch, size_t surviving_elites, size_t surviving_total, size_t mutations_per_epoch>
    class GeneticAlgorithm {

        static_assert(agents_in_epoch > surviving_total);
        static_assert(surviving_elites < surviving_total);

    public:

        /*!
         * Generates the initial random population
         */
        GeneticAlgorithm() {
            init();
        }

        /*!
         * Iterates entire epoch evaluation-selection-reproduction-mutation
         */
        void iterate_epoch() {
            next_epoch_agents_.clear();
            next_epoch_agents_.reserve(agents_in_epoch);

            agents_evaluation();
            selection();
            reproduction();
            mutation();

            agents_ = next_epoch_agents_;
        }

        /*!
         * Returns N-th best agent from the current epoch
         *
         * @param n - N-th best agent
         */
        AgentType best_agent(size_t n = 0) {
            agents_evaluation();
            sort_agents();
            return agents_.at(n).first;
        }

    private:

        /*!
         * Generates random population
         * */
        void init() {
            std::random_device r;
            engine_ = std::default_random_engine(r());
            distribution_ = std::uniform_real_distribution<float>(0, 1);

            agents_.reserve(agents_in_epoch);
            for (size_t i = 0 ; i < agents_in_epoch ; i++) {
                agents_.push_back({AgentType::random(), 0.0f});
            }
        }

        /*!
         * Evaluates current population, estimates score for each agent
         * */
        void agents_evaluation() {
            float cum_sum = 0.0f;

            for(auto& agent : agents_) {
                float score = agent.first.score();
                agent.second = score;
                cum_sum += score;
            }

            for(auto& agent : agents_) {agent.second /= cum_sum;}
        }

        /*!
         * Selects survivals to the next epoch
         * */
        void selection() {
            select_elites();
            select_random();
        }


        /*!
         * Select N best agents from current population, that will survive to the next epoch
         * */
        void select_elites() {
            sort_agents();
            for (size_t i = 0 ; i < surviving_elites ; i+=1) {
                next_epoch_agents_.push_back(agents_.at(i));
            }
        }

        /*!
         * Sort agents w.r.t. their score
         * */
        void sort_agents() {
            sort(agents_.begin(), agents_.end(), [](const std::pair<AgentType, float>& a, const std::pair<AgentType, float>& b) -> bool {
                return a.second > b.second;
            });
        }

        /*!
        * Randomly select M survivals. (Randomly selected + elites) = number of total survivals
        * */
        void select_random() {
            for (size_t i = next_epoch_agents_.size() ; i < surviving_total ; i+=1) {
                next_epoch_agents_.push_back(agents_.at(get_random_index(agents_.size())));
            }
        }

        /*!
         * Randomly mutate P agents. Agent can be mutated multiple times
         * Does not mutates the 1 best agent
         * */
        void mutation() {
            for (size_t i = 0 ; i < mutations_per_epoch ; i+=1) {
                // do not mutate the best agent
                next_epoch_agents_.at(get_random_index(next_epoch_agents_.size()-1)+1).first.mutate();
            }
        }

        /*!
         * Randomly selects pairs of survivals and generates new agents by crossing over the selected pair.
         * */
        void reproduction() {
            for (size_t i = next_epoch_agents_.size(); i < agents_in_epoch; i += 1) {
                auto rand_index_1 = get_random_index(agents_.size());
                auto rand_index_2 = get_random_index(agents_.size());
                next_epoch_agents_.push_back({agents_.at(rand_index_1).first.crossover(agents_.at(rand_index_2).first), 0.0f});
            }
        }

        /*!
         * Generates random index from 0 tp range-1
         * */
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
