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

#ifndef ROBOTICTEMPLATELIBRARY_KALMAN_H
#define ROBOTICTEMPLATELIBRARY_KALMAN_H

#include "rtl/core/Matrix.h"

namespace rtl
{

    /*!
     * Simple implementation of Kalman Filter with strict typed input and output matrices.
     * Before using the KF, user has to manually specify the inner matrices of the filter.
     *
     * @tparam dtype Data type of values in KF's matrices
     * @tparam state_dim Dimension of inner state vector
     * @tparam measurement_dim Dimension of measurement vector
     * @tparam control_dim Dimension of control vector
     */
    template <typename dtype, size_t state_dim, size_t measurement_dim, size_t control_dim>
    class Kalman {

    public:

        Kalman(dtype process_noise, dtype observation_noise) :
                process_noise_{process_noise},
                observation_noise_{observation_noise} {
            x_states_ = Matrix<state_dim, 1, dtype>::zeros();
            A_transition_matrix_ = Matrix<state_dim, state_dim, dtype>::identity();
            B_control_matrix_ = Matrix<state_dim, control_dim, dtype>::zeros();
            P_covariance_ = Matrix<state_dim, state_dim, dtype>::identity();
            H_measurement_matrix_ = Matrix<measurement_dim, state_dim, dtype>::zeros();

            Q_process_noise_covariance_ = Matrix<state_dim, state_dim, dtype>::zeros() * process_noise_;
            R_measurement_noise_covariance_ = Matrix<measurement_dim, measurement_dim, dtype>::identity() * observation_noise_;

            I_ = Matrix<state_dim, state_dim, dtype>::identity();
        }

        void predict(Matrix<control_dim, 1, dtype> control_input) {
            x_states_ = A_transition_matrix_ * x_states_ + B_control_matrix_ * control_input;
            P_covariance_ = A_transition_matrix_ * P_covariance_ * A_transition_matrix_.transposed() + Q_process_noise_covariance_;
        }

        void correct(Matrix<measurement_dim, 1, dtype> z_measurement) {
            K_kalman_gain_ = P_covariance_ * H_measurement_matrix_.transposed() * (H_measurement_matrix_ * P_covariance_ * H_measurement_matrix_.transposed() + R_measurement_noise_covariance_).inverted();
            auto tmp = (z_measurement - H_measurement_matrix_ * x_states_);
            x_states_ = x_states_ + K_kalman_gain_ * (z_measurement - H_measurement_matrix_ * x_states_);
            P_covariance_ = (I_ - K_kalman_gain_ * H_measurement_matrix_) * P_covariance_;
        }


        const Matrix<state_dim, 1, dtype>& states() const {return x_states_;}
        const Matrix<state_dim, state_dim, dtype>& covariance() const {return P_covariance_;}
        const Matrix<state_dim, measurement_dim, dtype>& kalman_gain() {return K_kalman_gain_;}

        void set_states(const Matrix<state_dim, 1, dtype>& states) {x_states_ = states;}
        void set_transision_matrix(const Matrix<state_dim, state_dim, dtype>& transition_matrix) {A_transition_matrix_ = transition_matrix;}
        void set_control_matrix(const Matrix<state_dim, control_dim, dtype>& control_matrix) {B_control_matrix_ = control_matrix;}
        void set_measurement_matrix(const Matrix<measurement_dim, state_dim, dtype>& measurement_matrix) {H_measurement_matrix_ = measurement_matrix;}

        void set_process_noise_covariance_matrix(const Matrix<state_dim, state_dim, dtype>& process_noise_covariance) {Q_process_noise_covariance_ = process_noise_covariance;}
        void set_measurement_noise_covariance_matrix(const Matrix<measurement_dim, measurement_dim, dtype>& measurement_noise_covariance) {R_measurement_noise_covariance_ = measurement_noise_covariance;}

    private:
        dtype process_noise_;
        dtype observation_noise_;

        Matrix<state_dim, 1, dtype> x_states_;
        Matrix<state_dim, state_dim, dtype> A_transition_matrix_;
        Matrix<state_dim, control_dim, dtype> B_control_matrix_;
        Matrix<state_dim, state_dim, dtype> P_covariance_;
        Matrix<state_dim, measurement_dim, dtype> K_kalman_gain_;
        Matrix<measurement_dim, state_dim, dtype> H_measurement_matrix_;

        Matrix<state_dim, state_dim, dtype> Q_process_noise_covariance_;
        Matrix<measurement_dim, measurement_dim, dtype> R_measurement_noise_covariance_;
        Matrix<state_dim, state_dim, dtype> I_;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_KALMAN_H
