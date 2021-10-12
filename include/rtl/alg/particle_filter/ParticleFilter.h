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

#include <rtl/alg/particle_filter/GenericParticle.h>
#include <rtl/alg/particle_filter/SimpleParticle.h>

namespace rtl
{
    template<typename ParticleType, size_t no_of_particles, size_t no_of_survivors>
    class ParticleFilter {


    public:

        ParticleFilter() {
            particles_and_cum_score.reserve(no_of_particles);
            for (size_t i = 0 ; i < no_of_particles ; i++) {
                particles_and_cum_score.push_back(std::pair<ParticleType, double>{ParticleType::random(), 0.0});
            }
        }


        void iteration(const ParticleType& action, const ParticleType& measurement) {
            prediction(action);
            correction(measurement);
            resampling();
        }

        ParticleType estimated_val() {
            std::vector<ParticleType> evaluation_particles;
            evaluation_particles.reserve(no_of_survivors);

            for (size_t i = 0 ; i < no_of_survivors ; i++) {
                evaluation_particles.push_back(particles_and_cum_score.at(i).first);
            }
            return ParticleType::evaluation(evaluation_particles);
        }

    private:

        void prediction(const ParticleType& action) {
            std::for_each(particles_and_cum_score.begin(), particles_and_cum_score.end(), [&](auto particle){
                particle.first.move(action);
            });
        }

        void correction(const ParticleType& measurement) {
            double cum_sum = 0.0;
            for (auto& particle_cum_score : particles_and_cum_score) {
                auto particle_score = particle_cum_score.first.belief(measurement);
                particle_cum_score.second = cum_sum + particle_score;
                cum_sum += particle_score;
            }

            normalize_score(cum_sum);
        }

        void normalize_score(double cum_sum) {
            std::for_each(particles_and_cum_score.begin(), particles_and_cum_score.end(), [&](auto& particle_score){
                particle_score.second /= cum_sum;
            });
        }

        void resampling() {
            std::vector<std::pair<ParticleType, double>> new_particles;
            new_particles.reserve(particles_and_cum_score.size());

            double step = 1.0 / (no_of_survivors+1);
            double th = 0.0;
            for (size_t n = 0 ; n < no_of_survivors ; n++) {
                th += step;
                for(auto it = particles_and_cum_score.begin() ; it < particles_and_cum_score.end() ; it++) {
                    if (it->second > th) {
                        new_particles.push_back(*it);
                        break;
                    }
                }
            }

            for (size_t n = new_particles.size() ; n < no_of_particles ; n++) {
                new_particles.push_back(std::pair<ParticleType, double>{ParticleType::random(), 0.0});
            }
            particles_and_cum_score = new_particles;
        }

        std::vector<std::pair<ParticleType, double>> particles_and_cum_score;
    };

}

#endif //ROBOTICTEMPLATELIBRARY_PARTICLEFILTER_H
