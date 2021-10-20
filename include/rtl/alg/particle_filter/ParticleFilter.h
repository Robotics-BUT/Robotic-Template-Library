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

#ifndef ROBOTICTEMPLATELIBRARY_PARTICLEFILTER_H
#define ROBOTICTEMPLATELIBRARY_PARTICLEFILTER_H

#include <rtl/alg/particle_filter/SimpleParticle.h>

namespace rtl {

    template<typename ParticleType, size_t no_of_particles, size_t no_of_survivors>
    class ParticleFilter {

        using score_type = float;

    public:

        ParticleFilter() {
            particles_.reserve(no_of_particles);
            for (size_t i = 0 ; i < no_of_particles ; i++) {
                particles_.push_back(std::pair<ParticleType, score_type>{ParticleType::random(), 0.0});
            }
        }


        void iteration(const typename ParticleType::Action& action, const typename ParticleType::Measurement& measurement) {
            prediction(action);
            correction(measurement);
            resampling();
        }

        typename ParticleType::Result evaluate() {
            std::vector<ParticleType> evaluation_particles;
            evaluation_particles.reserve(no_of_survivors);

            for (size_t i = 0 ; i < no_of_survivors ; i++) {
                evaluation_particles.push_back(particles_.at(i).first);
            }
            return ParticleType::evaluation(evaluation_particles);
        }

    private:

        void prediction(const typename ParticleType::Action& action) {
            std::for_each(particles_.begin(), particles_.end(), [&](auto& particle){
                particle.first.move(action);
            });
        }

        void correction(const typename ParticleType::Measurement& measurement) {
            double cum_sum = 0.0;
            for (auto& particle_cum_score : particles_) {
                auto particle_score = particle_cum_score.first.belief(measurement);
                particle_cum_score.second = cum_sum + particle_score;
                cum_sum += particle_score;
            }

            normalize_score(cum_sum);
        }

        void normalize_score(double cum_sum) {
            std::for_each(particles_.begin(), particles_.end(), [&](auto& particle_score){
                particle_score.second /= cum_sum;
            });
        }

        void resampling() {
            std::vector<std::pair<ParticleType, score_type>> new_particles;
            new_particles.reserve(particles_.size());

            double step = 1.0 / (no_of_survivors+1);
            double th = 0.0;

            auto it = particles_.begin();
            for (size_t n = 0 ; n < no_of_survivors ; n++) {
                th += step;
                while(true) {
                    if (it->second > th) {
                        new_particles.push_back(*it);
                        break;
                    } else {
                        it++;
                    }
                }
            }

            for (size_t n = new_particles.size() ; n < no_of_particles ; n++) {
                new_particles.push_back(std::pair<ParticleType, score_type>{ParticleType::random(), 0.0});
            }
            particles_ = new_particles;
        }

        std::vector<std::pair<ParticleType, score_type>> particles_;
    };

}

#endif //ROBOTICTEMPLATELIBRARY_PARTICLEFILTER_H
