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
#include "rtl/Core.h"
#include "rtl/Transformation.h"

class Robot {

public:

    class Action {
    public:
        explicit Action(const rtl::Vector2f& trans, const rtl::Rotation2f& rot) : translation_{trans}, rotation_{rot} {}
        [[nodiscard]] rtl::Vector2f translation() const {return translation_;}
        [[nodiscard]] rtl::Rotation2f rotation() const {return rotation_;}
    private:
        rtl::Vector2f translation_;
        rtl::Rotation2f rotation_;
    };

    class Measurement {
    public:
        explicit Measurement(const rtl::Vector2f& val) : value_{val} {}
        [[nodiscard]] rtl::Vector2f value() const {return value_;}
    private:
        rtl::Vector2f value_;
    };

    class Result {
    public:
        explicit Result(const rtl::Vector2f& mean_pose,
                        const rtl::Vector2f& std_dev_pose,
                        const rtl::Rotation2f& mean_orientation,
                        const rtl::Rotation2f& std_dev_orientation) : mean_pose_{mean_pose},
                                                   std_dev_pose_{std_dev_pose},
                                                   mean_orientation_{mean_orientation},
                                                   std_dev_orientation_{std_dev_orientation} {}
        [[nodiscard]] rtl::Vector2f mean_pose() const {return mean_pose_;}
        [[nodiscard]] rtl::Vector2f std_dev_pose() const {return std_dev_pose_;}
        [[nodiscard]] rtl::Rotation2f mean_orientation() const {return mean_orientation_;}
        [[nodiscard]] rtl::Rotation2f std_dev_orientation() const {return std_dev_orientation_;}
    private:
        rtl::Vector2f mean_pose_;
        rtl::Vector2f std_dev_pose_;
        rtl::Rotation2f mean_orientation_;
        rtl::Rotation2f std_dev_orientation_;
    };


    explicit Robot(const rtl::Vector2f& pose, const rtl::Rotation2f& orientation) : pose_{pose}, orientation_{orientation} {}

    static Robot random() {
        return Robot(rtl::Vector2f{get_random_float(-10.0f, 10.0f),
                                         get_random_float(-10.0f, 10.0f)},
                     rtl::Rotation2f{get_random_float(0.0f, 2.0f*M_PIf32)});
    }

    void move(const Robot::Action& action) {
        pose_ += action.translation().transformed(orientation_);
        orientation_.transform(action.rotation());
    }

    [[nodiscard]] float belief(const Robot::Measurement& measurement) {
        return gauss(cost(measurement));
    }

    [[nodiscard]] static Result evaluation(const std::vector<Robot>& vec) {
        auto sum = rtl::Vector2f::zeros();
        auto sin_sum = 0.0f;
        auto cos_sum = 0.0f;
        std::for_each(vec.begin(), vec.end(), [&](const Robot& particle){
            sum += particle.pose_;
            sin_sum += particle.orientation_.rotSin();
            cos_sum += particle.orientation_.rotCos();
        });

        auto mean_pose = sum / vec.size();
        auto square_diff_sum = rtl::Vector2f::zeros();

        float mean_angle = std::atan2(sin_sum / vec.size(),
                                      cos_sum / vec.size());
        auto square_diff_angle_sum = 0.0f;

        std::for_each(vec.begin(), vec.end(), [&](const Robot& particle){
            square_diff_sum += rtl::Vector2f{powf(particle.pose_.x() - mean_pose.x(), 2.0f),
                                             powf(particle.pose_.y() - mean_pose.y(), 2.0f)};
            auto ang_diff = mean_angle-particle.orientation_.rotAngle();
            square_diff_angle_sum = powf(std::atan2(std::sin(ang_diff), std::cos(ang_diff)), 2.0f);
        });
        auto std_dev_pose = rtl::Vector2f{std::sqrt(square_diff_sum.x() / vec.size()),
                                          std::sqrt(square_diff_sum.y() / vec.size())};
        float std_dev_angle = std::sqrt(square_diff_angle_sum / vec.size());

        return Result(mean_pose, std_dev_pose, rtl::Rotation2f(mean_angle), rtl::Rotation2f(std_dev_angle));
    }

private:

    [[nodiscard]] float cost(const Robot::Measurement& measurement) {
        return std::sqrt( std::pow(pose_.x()-measurement.value().x(), 2.0f) +
                          std::pow(pose_.y()-measurement.value().y(), 2.0f));
    }

    static float get_random_float(float min, float max) {
        static std::random_device r;
        static auto engine = std::default_random_engine(r());
        static std::uniform_real_distribution<float> distribution(min, max);
        return distribution(engine);
    }

    static float gauss(float x) {
        constexpr float mean = 0;
        constexpr float std_dev = 1;
        constexpr float variance = std_dev * std_dev;
        constexpr float sqrt_2_pi = 2.5066;
        constexpr float a = (1 / (std_dev * sqrt_2_pi));
        return a * expf(-0.5f * powf(x - mean, 2) / variance);
    }

    rtl::Vector2f pose_;
    rtl::Rotation2f orientation_;
};

int main() {

    auto robot_pose = rtl::Vector2f{0.0, 0.0};
    auto robot_orientation = rtl::Rotation2f{0.0f};

    const auto motion_translation = rtl::Vector2f{1.0, 0.0};
    const auto motion_rotation = rtl::Rotation2f{M_PI/2.0f / 9}; // 10 deg

    std::cout << "pose: " << robot_pose.x() << " " << robot_pose.y() << " orient: " << robot_orientation.rotAngle() << std::endl;
    auto particle_filter = rtl::ParticleFilter<Robot, 500, 300>();
    for (size_t i = 0 ; i < 36 ; i++) {

        // Oriented translation, based on current robot rotation
        auto oriented_translation = motion_translation.transformed(robot_orientation);

        // Robot first translates, than rotates
        robot_pose += oriented_translation;
        robot_orientation.transform(motion_rotation);
        std::cout << "- - - - - - - - - - - - - - - - - - - -" << std::endl;
        std::cout << "GT pose: " << robot_pose.x() << " " << robot_pose.y() << " orient: " << robot_orientation.rotAngle() << std::endl;

        particle_filter.iteration(Robot::Action{motion_translation, motion_rotation}, Robot::Measurement{robot_pose});
        auto result = particle_filter.evaluate();
        std::cout << "Estimated pose x: " << result.mean_pose().x() << " std_dev: " << result.std_dev_pose().x() << std::endl
                  << "          pose y: " << result.mean_pose().y() << " std_dev: " << result.std_dev_pose().y() << std::endl
                  << "     orientation: " << result.mean_orientation().rotAngle() << " std_dev: " << result.std_dev_orientation().rotAngle() << std::endl;
    }

    return 0;
}