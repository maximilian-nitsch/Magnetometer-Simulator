/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <Eigen/Dense>

namespace mag_simulator {

struct StochasticErrors {
  Eigen::Vector3d z_N;  // "Velocity" random walk
  Eigen::Vector3d z_B;  // Bias instability
  Eigen::Vector3d z_K;  // "Acceleration" random walk
};

struct MagSimParams {
  Eigen::Vector3d N;             // "Velocity" random walk (T/s^(1/2))
  Eigen::Vector3d B;             // Bias instability (T/s)
  Eigen::Vector3d K;             // "Acceleration" random walk (T/s^(3/2)))
  Eigen::Vector3d corrTime;      // Correlation time (s)
  Eigen::Vector3d hardIronBias;  // (T)
  Eigen::Matrix3d softIronDistortion;  // (-)
  Eigen::Vector3d measRange;           // Measurement range (T)
  Eigen::Vector3d resolution;          // Measurement resolution (T/LSB)
  Eigen::Matrix3d
      sensorRotation;  // Sensor rotation matrix (from sensor to body frame) (-)
};

struct ModelEnableSettings {
  bool enableHardIronBias;        // on/off hard iron bias
  bool enableSoftIronDistortion;  // on/off soft iron distortion
  bool enableStochasticError;     // on/off colored noise (stochastic error)
  bool enableSaturation;          // on/off saturation given a measurement range
  bool enableQuantization;        // on/off quantization given a resolution
};

// Identifier for measurement axis XYZ
enum AxisIdentifier : int { axisX = 0, axisY = 1, axisZ = 2 };

}  // namespace mag_simulator
