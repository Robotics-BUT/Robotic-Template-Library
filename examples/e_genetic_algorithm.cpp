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

#include <iostream>
#include "rtl/Algorithms.h"

// Implementing my own agent class, that represents string and implements necessary public methods:

class StringAgent {
public:

    explicit StringAgent(const std::string& val) : value_{val} {}

    static StringAgent random() {
        std::string val = "";
        val.resize(22);
        for (size_t i = 0 ; i < 22 ; i++) {
            val.at(i) = char_set.at(get_random_index(char_set.length()));
        }
        return StringAgent(val);
    }

    [[nodiscard]] std::string value() const {
        return value_;
    }

    float score() {
        return fit_(value_);
    }

    [[nodiscard]] StringAgent crossover(const StringAgent& mate) {
        std::string val;
        val.resize(22);

        for (size_t i = 0 ; i < 22 ; i++) {
            val.at(i) = get_random_index(1) ? value_.at(i) : mate.value_.at(i);
        }

        return StringAgent(val);
    }

    void mutate() {
        auto char_index = get_random_index(value_.length());
        value_.at(char_index) = char_set.at(get_random_index(char_set.length()));
    }

    static void set_fit_fn(const std::function<float(std::string)>& fn) {
        fit_ = fn;
    }

private:

    inline static std::function<float(const std::string)> fit_ = [](const std::string& s){
        size_t score = 0;
        const std::string target = "RoboticTemplateLibrary";
        assert(s.length() == target.length());

        for (size_t i = 0 ; i < target.length() ; i+=1) {
            if (target.at(i) == s.at(i)) {score+=1;}
        }

        return score;
    };

    static size_t get_random_index(size_t range) {
        static std::random_device r;
        static auto engine = std::default_random_engine(r());
        std::uniform_real_distribution<float> distribution(0, range);
        return static_cast<size_t>(distribution(engine));
    }

    std::string value_;
    static std::string char_set;
};
std::string StringAgent::char_set = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ ";


int main() {

    // Setup the genetic algorithm (100 agenst per epoch, 20 elites, 60 epoch survivals, 10 mutations per epoch)
    auto genetic_algorithm = rtl::GeneticAlgorithm<StringAgent, 100, 20, 60, 10>();

    // Run 1000 epochs
    for (size_t i = 0 ; i < 1000 ; i++) {
        genetic_algorithm.iterate_epoch();
        auto best = genetic_algorithm.best_agent();
        //std::cout << "Epoch no. " << i << " best: " << best.value() << " score: " << best.score() << std::endl;
        if (best.score() == 22) {break;}
    }

    // Print result
    auto best = genetic_algorithm.best_agent();
    std::cout << "Result: " << best.value() << std::endl;

    // Reinitialize
    genetic_algorithm = rtl::GeneticAlgorithm<StringAgent, 100, 20, 60, 10>();

    // Set new cost function
    StringAgent::set_fit_fn([](const std::string& s){
        size_t score = 0;
        const std::string target = "RTL is super cool libr";
        assert(s.length() == target.length());

        for (size_t i = 0 ; i < target.length() ; i+=1) {
            if (target.at(i) == s.at(i)) {score+=1;}
        }

        return score;
    });

    // Run 1000 epochs
    for (size_t i = 0 ; i < 1000 ; i++) {
        genetic_algorithm.iterate_epoch();
        auto best = genetic_algorithm.best_agent();
        //std::cout << "Epoch no. " << i << " best: " << best.value() << " score: " << best.score() << std::endl;
        if (best.score() == 22) {break;}
    }

    // Print results
    best = genetic_algorithm.best_agent();
    std::cout << "Result: " << best.value() << std::endl;

    return 0;
}