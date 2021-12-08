#include <gtest/gtest.h>
#include <rtl/Core.h>
#include <math.h>

#include "rtl/Algorithms.h"

#define max_err_1 1e-1
#define max_err_2 1e-2
#define max_err_10 1e-10


TEST(t_kalman, init) {
    auto filter = rtl::Kalman<float, 2, 2, 1>(0.1, 0.1);
}

TEST(t_kalman, filtration_1) {
    auto filter = rtl::Kalman<float, 2, 2, 1>(0.1, 0.1);

    float speed = 1.0f;
    float pose = 0.0f;
    float dt_step = 1.0f;

    for (size_t i = 0 ; i < 100 ; i++) {

        pose += speed * dt_step;

        filter.predict(rtl::Matrix<1,1,float>::zeros());

        auto measurement = rtl::Matrix<2, 1, float>::zeros();
        measurement.setElement(0,0, pose);
        measurement.setElement(1, 0, speed);
        filter.correct(measurement);
    }

    EXPECT_EQ(filter.states().getElement(0, 0), 0);
    EXPECT_EQ(filter.states().getElement(1, 0), 0);
}


TEST(t_kalman, filtration_2) {

    float speed = 1.0f;
    float pose = 0.0f;
    float dt_step = 1.0f;


    auto filter = rtl::Kalman<float, 2, 2, 1>(0.1, 0.1);
    filter.set_measurement_matrix(rtl::Matrix<2, 2, float>::identity());
    auto A = rtl::Matrix<2, 2, float>::zeros();
    A.setElement(0, 0, 1.0f);
    A.setElement(0, 1, dt_step);
    A.setElement(1, 1, 1.0f);
    filter.set_transision_matrix(A);

    for (size_t i = 0 ; i <= 100 ; i++) {
        pose += speed * dt_step;
        filter.predict(rtl::Matrix<1,1,float>::zeros());

        auto measurement = rtl::Matrix<2, 1, float>::zeros();
        measurement.setElement(0,0, pose);
        measurement.setElement(1, 0, speed);
        filter.correct(measurement);

    }

    EXPECT_NEAR(filter.states().getElement(0, 0), pose, max_err_1);
    EXPECT_NEAR(filter.states().getElement(1, 0), speed, max_err_1);
}


TEST(t_kalman, filtration_3) {

    float acc = 1.0f;
    float speed = 0.0f;
    float pose = 0.0f;
    float dt_step = 0.1f;

    float process_noise = 0.01;
    float observation_noise = 0.01;

    auto filter = rtl::Kalman<float, 3, 1, 1>(process_noise, observation_noise);

    auto A = rtl::Matrix<3, 3, float>::zeros();
    A.setRow(0, rtl::Vector3f{1.0f, dt_step, 0.5f*dt_step*dt_step});
    A.setElement(0,0,1);A.setElement(0,1,dt_step);A.setElement(0,2,0.5*acc*dt_step*dt_step);

    A.setRow(1, rtl::Vector3f{0.0f, 1   , dt_step});
    A.setElement(1,0,0); A.setElement(1,1,1); A.setElement(1,2,dt_step);
    A.setRow(2, rtl::Vector3f{0.0f, 0.0f, 1});
    A.setElement(2,0,0); A.setElement(2,1,0); A.setElement(2,2,1);
    filter.set_transision_matrix(A);

    auto H = rtl::Matrix<1, 3, float>::zeros();
    H.setElement(0, 0, 1.0f);
    filter.set_measurement_matrix(H);

    auto B = rtl::Matrix<3,1,float>::zeros();
    B.setElement(0, 0, 0.5f*dt_step*dt_step);
    B.setElement(1, 0, dt_step);
    B.setElement(2, 0, 0);
    filter.set_control_matrix(B);

    auto Q = rtl::Matrix<3,3,float>::zeros();
    Q.setRow(0, rtl::Vector3f{powf(dt_step, 6) * process_noise, powf(dt_step, 5) * process_noise, powf(dt_step, 4) * process_noise});
    Q.setRow(1, rtl::Vector3f{powf(dt_step, 5) * process_noise, powf(dt_step, 4) * process_noise, powf(dt_step, 3) * process_noise});
    Q.setRow(2, rtl::Vector3f{powf(dt_step, 4) * process_noise, powf(dt_step, 3) * process_noise, powf(dt_step, 2) * process_noise});
    filter.set_process_noise_covariance_matrix(Q);

    auto R = rtl::Matrix<1,1,float>::zeros();
    R.setElement(0, 0, observation_noise);
    filter.set_measurement_noise_covariance_matrix(R);


    for (size_t i = 0 ; i < 100 ; i++) {

        pose += speed * dt_step;
        speed += acc * dt_step;

        auto control = rtl::Matrix<1,1,float>::zeros();
        control.setElement(0,0, acc);
        filter.predict(control);

        auto measurement = rtl::Matrix<1, 1, float>::zeros();
        measurement.setElement(0,0, pose);
        filter.correct(measurement);
    }

    EXPECT_NEAR(filter.states().getElement(0, 0), pose, max_err_1);
    EXPECT_NEAR(filter.states().getElement(1, 0), speed, max_err_1);
}

int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
