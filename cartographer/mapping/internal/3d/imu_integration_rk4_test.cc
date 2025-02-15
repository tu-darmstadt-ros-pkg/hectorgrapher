/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Tests based on
 * https://github.com/ethz-asl/maplab/blob/master/algorithms/imu-integrator-rk4/test/test_imu_integrator_basic_test.cc
 *
 */

#include "cartographer/mapping/internal/3d/imu_integration.h"


#ifdef WITH_RK4
#include "imu-integrator/imu-integrator.h"


#include <map>
#include <random>
#include <tuple>

#include "gmock/gmock.h"

#include "cartographer/mapping/value_conversion_tables.h"

namespace cartographer {
namespace mapping {
namespace {
constexpr double kPrecision = 1e-8;

Eigen::Vector4d getOrientationFromState(
    const Eigen::Matrix<double, imu_integrator::kStateSize, 1>& state) {
  return state.block<imu_integrator::kStateOrientationBlockSize, 1>(
      imu_integrator::kStateOrientationOffset, 0);
}
Eigen::Vector3d getGyroBiasFromState(
    const Eigen::Matrix<double, imu_integrator::kStateSize, 1>& state) {
  return state.block<imu_integrator::kGyroBiasBlockSize, 1>(
      imu_integrator::kStateGyroBiasOffset, 0);
}
Eigen::Vector3d getVelocityFromState(
    const Eigen::Matrix<double, imu_integrator::kStateSize, 1>& state) {
  return state.block<imu_integrator::kVelocityBlockSize, 1>(
      imu_integrator::kStateVelocityOffset, 0);
}
Eigen::Vector3d getAccelBiasFromState(
    const Eigen::Matrix<double, imu_integrator::kStateSize, 1>& state) {
  return state.block<imu_integrator::kAccelBiasBlockSize, 1>(
      imu_integrator::kStateAccelBiasOffset, 0);
}
Eigen::Vector3d getPositionFromState(
    const Eigen::Matrix<double, imu_integrator::kStateSize, 1>& state) {
  return state.block<imu_integrator::kPositionBlockSize, 1>(
      imu_integrator::kStatePositionOffset, 0);
}

TEST(IMUIntegrationRK4Test, GravityOnly) {
  imu_integrator::ImuIntegratorRK4 integrator(0.0, 0.0, 0.0, 0.0, 1.0);
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> current_state;
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> next_state;
  Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>
      debiased_imu_readings;
  current_state.setZero();
  current_state.block<imu_integrator::kStateOrientationBlockSize, 1>(
      imu_integrator::kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);
  debiased_imu_readings.setZero();
  Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                imu_integrator::kErrorStateSize>
      next_phi;
  Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                imu_integrator::kErrorStateSize>
      next_cov;
  double delta_time_seconds = 1.0;
  integrator.integrate(current_state, debiased_imu_readings, delta_time_seconds,
                       &next_state, &next_phi, &next_cov);
  double expected_translation = -1.0 * pow(delta_time_seconds, 2.0) / 2;
  double expected_final_velocity = -1.0 * delta_time_seconds;
  EXPECT_NEAR(
      (getOrientationFromState(next_state) - Eigen::Vector4d(0, 0, 0, 1))
          .norm(),
      0, 1e-15);
  EXPECT_NEAR(getGyroBiasFromState(next_state).norm(), 0, 1e-15);
  EXPECT_NEAR((getVelocityFromState(next_state) -
               Eigen::Vector3d(0, 0, expected_final_velocity))
                  .norm(),
              0, 1e-15);
  EXPECT_NEAR(getAccelBiasFromState(next_state).norm(), 0, 1e-15);
  EXPECT_NEAR((getPositionFromState(next_state) -
               Eigen::Vector3d(0, 0, expected_translation))
                  .norm(),
              0, 1e-15);
  EXPECT_NEAR(next_cov.norm(), 0, 1e-15);
}

TEST(IMUIntegrationRK4Test, ConstantAcceleration) {
  Eigen::Vector3d initial_gravity_acceleration(0, 0, 10);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time start_time = common::FromUniversal(0);
  common::Time current_time = start_time;

  imu_integrator::ImuIntegratorRK4 integrator(0.0, 0.0, 0.0, 0.0, 0);
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> current_state;
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> next_state;
  Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>
      debiased_imu_readings;
  current_state.setZero();
  current_state.block<imu_integrator::kStateOrientationBlockSize, 1>(
      imu_integrator::kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);
  debiased_imu_readings.setZero();
  debiased_imu_readings.block<3, 1>(imu_integrator::kAccelReadingOffset, 0) =
      Eigen::Vector3d(0.0, 0, 10.0);
  debiased_imu_readings.block<3, 1>(
      imu_integrator::kImuReadingSize + imu_integrator::kAccelReadingOffset,
      0) = Eigen::Vector3d(0, 0, 10.0);
  for (int i = 0; i < 100; ++i) {
    current_time += common::FromSeconds(1);

    integrator.integrateStateOnly(current_state, debiased_imu_readings, 1.0,
                                  &next_state);
    current_state = next_state;

    double delta_time = common::ToSeconds(current_time - start_time);
    Eigen::Vector3d expected_velocity =
        delta_time * initial_gravity_acceleration;
    Eigen::Vector3d expected_translation =
        0.5 * delta_time * delta_time * initial_gravity_acceleration;
    EXPECT_NEAR((getVelocityFromState(next_state) - expected_velocity).norm(),
                0, kPrecision);
    EXPECT_NEAR(
        (getPositionFromState(next_state) - expected_translation).norm(), 0,
        kPrecision);
    LOG(INFO)
        << "expected \t" << (expected_translation).norm() << " observed \t"
        << (getPositionFromState(next_state)).norm() << " abs error \t"
        << (getPositionFromState(next_state) - expected_translation).norm();
  }
}

TEST(IMUIntegrationRK4Test, StandingInGravity) {
  Eigen::Vector3d initial_gravity_acceleration(0, 0, 10);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time start_time = common::FromUniversal(0);
  common::Time current_time = start_time;

  imu_integrator::ImuIntegratorRK4 integrator(0.0, 0.0, 0.0, 0.0, 10);
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> current_state;
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> next_state;
  Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>
      debiased_imu_readings;
  current_state.setZero();
  current_state.block<imu_integrator::kStateOrientationBlockSize, 1>(
      imu_integrator::kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);
  debiased_imu_readings.setZero();
  debiased_imu_readings.block<3, 1>(imu_integrator::kAccelReadingOffset, 0) =
      Eigen::Vector3d(0.0, 0, 10.0);
  debiased_imu_readings.block<3, 1>(
      imu_integrator::kImuReadingSize + imu_integrator::kAccelReadingOffset,
      0) = Eigen::Vector3d(0, 0, 10.0);
  for (int i = 0; i < 100; ++i) {
    current_time += common::FromSeconds(1);

    integrator.integrateStateOnly(current_state, debiased_imu_readings, 1.0,
                                  &next_state);
    current_state = next_state;
    Eigen::Vector3d expected_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d expected_translation = Eigen::Vector3d::Zero();
    EXPECT_NEAR((getVelocityFromState(next_state) - expected_velocity).norm(),
                0, kPrecision);
    EXPECT_NEAR(
        (getPositionFromState(next_state) - expected_translation).norm(), 0,
        kPrecision);
  }
}

TEST(IMUIntegrationRK4Test, ConstantAccelerationIMUObservations) {
  Eigen::Vector3d initial_gravity_acceleration(0, 0, 9.80665);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time start_time = common::FromUniversal(0);
  common::Time current_time = start_time;

  std::deque<sensor::ImuData> imu_data_deque;
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});
  current_time += common::FromSeconds(1);
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});
  current_time += common::FromSeconds(1);
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});

  common::Time integration_start_time =
      common::FromUniversal(0) + common::FromSeconds(0.5);
  common::Time integration_end_time =
      common::FromUniversal(0) + common::FromSeconds(1.5);
  auto it = --imu_data_deque.cend();
  while (it->time > integration_start_time) {
    CHECK(it != imu_data_deque.cbegin());
    --it;
  }

  State initial_state =
      State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero());
  ImuIntegrator imu_integrator(proto::IMUIntegrator::RK4);
  IntegrateImuWithTranslationResult<double> result =
      imu_integrator.IntegrateStateWithGravity(
          imu_data_deque, initial_state, start_time, integration_end_time, &it);
  Eigen::Vector3d expected_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d expected_translation = Eigen::Vector3d::Zero();
  EXPECT_NEAR((result.delta_velocity - expected_velocity).norm(), 0,
              kPrecision);
  EXPECT_NEAR((result.delta_translation - expected_translation).norm(), 0,
              kPrecision);
}

TEST(IMUIntegrationRK4Test, StandingInGravityIMUObservations) {
  Eigen::Vector3d observed_acceleration(0, 0, 0);
  Eigen::Vector3d gravity_acceleration(0, 0, 9.80665);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time start_time = common::FromUniversal(0);
  common::Time current_time = start_time;

  std::deque<sensor::ImuData> imu_data_deque;
  imu_data_deque.push_back(
      {current_time, observed_acceleration, initial_angular_velocity});
  current_time += common::FromSeconds(1);
  imu_data_deque.push_back(
      {current_time, observed_acceleration, initial_angular_velocity});
  current_time += common::FromSeconds(1);
  imu_data_deque.push_back(
      {current_time, observed_acceleration, initial_angular_velocity});
  current_time += common::FromSeconds(1);
  imu_data_deque.push_back(
      {current_time, observed_acceleration, initial_angular_velocity});

  common::Time integration_start_time =
      common::FromUniversal(0) + common::FromSeconds(0.25);
  common::Time integration_end_time =
      common::FromUniversal(0) + common::FromSeconds(0.5);
  auto it = --imu_data_deque.cend();
  while (it->time > integration_start_time) {
    CHECK(it != imu_data_deque.cbegin());
    --it;
  }

  State initial_state =
      State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero());

  ImuIntegrator imu_integrator(proto::IMUIntegrator::RK4);
  IntegrateImuWithTranslationResult<double> result =
      imu_integrator.IntegrateStateWithGravity(imu_data_deque, initial_state,
                                               integration_start_time,
                                               integration_end_time, &it);
  double delta_time =
      common::ToSeconds(integration_end_time - integration_start_time);
  Eigen::Vector3d expected_velocity = -delta_time * gravity_acceleration;
  Eigen::Vector3d expected_translation =
      -0.5 * delta_time * delta_time * gravity_acceleration;
  EXPECT_NEAR((result.delta_velocity - expected_velocity).norm(), 0,
              kPrecision);
  EXPECT_NEAR((result.delta_translation - expected_translation).norm(), 0,
              kPrecision);

  integration_start_time = common::FromUniversal(0) + common::FromSeconds(0.5);
  integration_end_time = common::FromUniversal(0) + common::FromSeconds(1.5);
  it = --imu_data_deque.cend();
  while (it->time > integration_start_time) {
    CHECK(it != imu_data_deque.cbegin());
    --it;
  }
  initial_state = State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                        Eigen::Vector3d::Zero());

  result = imu_integrator.IntegrateStateWithGravity(
      imu_data_deque, initial_state, integration_start_time,
      integration_end_time, &it);
  delta_time = common::ToSeconds(integration_end_time - integration_start_time);
  expected_velocity = -delta_time * gravity_acceleration;
  expected_translation = -0.5 * delta_time * delta_time * gravity_acceleration;
  EXPECT_NEAR((result.delta_velocity - expected_velocity).norm(), 0,
              kPrecision);
  EXPECT_NEAR((result.delta_translation - expected_translation).norm(), 0,
              kPrecision);

  integration_start_time = common::FromUniversal(0) + common::FromSeconds(0.5);
  integration_end_time = common::FromUniversal(0) + common::FromSeconds(2.5);
  it = --imu_data_deque.cend();
  while (it->time > integration_start_time) {
    CHECK(it != imu_data_deque.cbegin());
    --it;
  }
  initial_state = State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                        Eigen::Vector3d::Zero());
  result = imu_integrator.IntegrateStateWithGravity(
      imu_data_deque, initial_state, integration_start_time,
      integration_end_time, &it);
  delta_time = common::ToSeconds(integration_end_time - integration_start_time);
  expected_velocity = -delta_time * gravity_acceleration;
  expected_translation = -0.5 * delta_time * delta_time * gravity_acceleration;
  EXPECT_NEAR((result.delta_velocity - expected_velocity).norm(), 0,
              kPrecision);
  EXPECT_NEAR((result.delta_translation - expected_translation).norm(), 0,
              kPrecision);

  integration_start_time = common::FromUniversal(0) + common::FromSeconds(0.0);
  integration_end_time = common::FromUniversal(0) + common::FromSeconds(3.0);
  it = --imu_data_deque.cend();
  while (it->time > integration_start_time) {
    CHECK(it != imu_data_deque.cbegin());
    --it;
  }
  initial_state = State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                        Eigen::Vector3d::Zero());
  result = imu_integrator.IntegrateStateWithGravity(
      imu_data_deque, initial_state, integration_start_time,
      integration_end_time, &it);
  delta_time = common::ToSeconds(integration_end_time - integration_start_time);
  expected_velocity = -delta_time * gravity_acceleration;
  expected_translation = -0.5 * delta_time * delta_time * gravity_acceleration;
  EXPECT_NEAR((result.delta_velocity - expected_velocity).norm(), 0,
              kPrecision);
  EXPECT_NEAR((result.delta_translation - expected_translation).norm(), 0,
              kPrecision);
}

TEST(IMUIntegrationRK4Test,
     ConstandVelocityConstantAccelerationIMUObservations) {
  Eigen::Vector3d initial_gravity_acceleration(0, 0, 9.80665);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time start_time = common::FromUniversal(0);
  common::Time current_time = start_time;

  std::deque<sensor::ImuData> imu_data_deque;
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});
  current_time += common::FromSeconds(1);
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});
  current_time += common::FromSeconds(1);
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});

  common::Time integration_start_time =
      common::FromUniversal(0) + common::FromSeconds(0.25);
  common::Time integration_end_time =
      common::FromUniversal(0) + common::FromSeconds(1.75);
  auto it = --imu_data_deque.cend();
  while (it->time > integration_start_time) {
    CHECK(it != imu_data_deque.cbegin());
    --it;
  }

  State initial_state = State(Eigen::Vector3d::Zero(),
                              Eigen::Quaterniond::Identity(), {1.0, 0.0, 0.0});
  ImuIntegrator imu_integrator(proto::IMUIntegrator::RK4);
  IntegrateImuWithTranslationResult<double> result =
      imu_integrator.IntegrateStateWithGravity(imu_data_deque, initial_state,
                                               integration_start_time,
                                               integration_end_time, &it);
  Eigen::Vector3d expected_velocity = {1.0, 0.0, 0.0};
  Eigen::Vector3d expected_translation = {1.5, 0.0, 0.0};
  EXPECT_NEAR((result.delta_velocity - expected_velocity).norm(), 0,
              kPrecision);
  EXPECT_NEAR((result.delta_translation - expected_translation).norm(), 0,
              kPrecision);
}

TEST(IMUIntegrationTest, ConstantAccelerationIMUIntegrator) {
  Eigen::Vector3d initial_gravity_acceleration(0, 0, 10);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time start_time = common::FromUniversal(0);
  common::Time current_time = start_time;
  std::deque<sensor::ImuData> imu_data_deque;
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});
  for (int i = 0; i < 100; ++i) {
    current_time += common::FromSeconds(1);
    imu_data_deque.push_back(
        {current_time, initial_gravity_acceleration, initial_angular_velocity});

    auto it = --imu_data_deque.cend();
    while (it->time > start_time) {
      CHECK(it != imu_data_deque.cbegin());
      --it;
    }

    ImuIntegrator imu_integrator(proto::IMUIntegrator::RK4);
    IntegrateImuWithTranslationResult<double> result =
        imu_integrator.IntegrateIMU(imu_data_deque, start_time, current_time,
                                    &it);
    double delta_time = common::ToSeconds(current_time - start_time);
    Eigen::Vector3d expected_velocity =
        delta_time * initial_gravity_acceleration;
    Eigen::Vector3d expected_translation =
        0.5 * delta_time * delta_time * initial_gravity_acceleration;
    EXPECT_NEAR(0.f, (result.delta_velocity - expected_velocity).norm(),
                kPrecision);
    EXPECT_NEAR(0.f, (result.delta_translation - expected_translation).norm(),
                delta_time * 10);
    EXPECT_NEAR(
        0.,
        result.delta_rotation.angularDistance(Eigen::Quaterniond::Identity()),
        kPrecision);
    LOG(INFO) << "xexpected \t" << (expected_translation).norm()
              << " observed \t" << (result.delta_translation).norm()
              << " abs error \t"
              << (result.delta_translation - expected_translation).norm();
  }
}

TEST(IMUIntegrationTest, ConstantAccelerationIMUIntegratorWithGravity) {
  Eigen::Vector3d initial_gravity_acceleration(0, 0, 10);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time start_time = common::FromUniversal(0);
  common::Time current_time = start_time;
  std::deque<sensor::ImuData> imu_data_deque;
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});
  for (int i = 0; i < 100; ++i) {
    current_time += common::FromSeconds(1);
    imu_data_deque.push_back(
        {current_time, initial_gravity_acceleration, initial_angular_velocity});

    auto it = --imu_data_deque.cend();
    while (it->time > start_time) {
      CHECK(it != imu_data_deque.cbegin());
      --it;
    }

    ImuIntegrator imu_integrator(proto::IMUIntegrator::RK4);
    imu_integrator.SetGravityAcceleration(10.0);
    IntegrateImuWithTranslationResult<double> result =
        imu_integrator.IntegrateIMU(imu_data_deque, start_time, current_time,
                                    &it);
    double delta_time = common::ToSeconds(current_time - start_time);
    Eigen::Vector3d expected_velocity =
        delta_time * initial_gravity_acceleration;
    Eigen::Vector3d expected_translation =
        0.5 * delta_time * delta_time * initial_gravity_acceleration;
    EXPECT_NEAR(0.f, (result.delta_velocity - expected_velocity).norm(),
                kPrecision);
    EXPECT_NEAR(0.f, (result.delta_translation - expected_translation).norm(),
                delta_time * 10);
    EXPECT_NEAR(
        0.,
        result.delta_rotation.angularDistance(Eigen::Quaterniond::Identity()),
        kPrecision);
    LOG(INFO) << "gexpected \t" << (expected_translation).norm()
              << " observed \t" << (result.delta_translation).norm()
              << " abs error \t"
              << (result.delta_translation - expected_translation).norm();
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer

#endif
