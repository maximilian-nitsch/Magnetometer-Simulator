/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <memory>
#include <random>

#include "mag_simulator_structures.h"

namespace mag_simulator {

class MagSimulator {
 public:
  // Constructor
  explicit MagSimulator(
      Eigen::Vector3d localMagFieldVector = Eigen::Vector3d::Zero(),
      MagSimParams magSimParams = MagSimParams(),
      ModelEnableSettings modelEnableSettings = ModelEnableSettings(),
      double dt = 0.01, unsigned int seed = 42);

  // Destructor
  ~MagSimulator();

  // Measurement generation function
  Eigen::Vector3d generateMagnetometerMeasurement(
      const Eigen::Quaterniond q_b_n_true);

  // Reset function to reset Gauss-Markov processes
  void resetSimulator();

  // Getter functions
  Eigen::Vector3d getLocalMagFieldVector() const;
  MagSimParams getMagSimParams() const;
  unsigned int getSimulatorSeed() const;
  double getMagSampleTime() const;
  ModelEnableSettings getModelEnableSettings() const;
  Eigen::Vector3d getHardIronBias() const;
  Eigen::Matrix<double, 3, 3> getSoftIronDistortion() const;
  StochasticErrors getStochasticErrors() const;
  bool getUseFixedRandomNumbersFlag() const;

  // Setter functions
  void setLocalMagFieldVector(const Eigen::Vector3d& localMagFieldVector);
  void setMagSimParams(const MagSimParams& magSimParams);
  void setSimulatorSeed(const unsigned int seed);
  void setMagSampleTime(const double dt);
  void setModelEnableSettings(const ModelEnableSettings& modelEnableSettings);
  void setEnableHardIronBias(const bool enable);
  void setEnableSoftIronDistortion(const bool enable);
  void setEnableStochasticError(const bool enable);
  void setEnableSaturation(const bool enable);
  void setEnableQuantization(const bool enable);
  void setUseFixedRandomNumbersFlag(const bool enable);

  // Print functions
  std::stringstream printMagSimulatorParameters();

 private:
  // Local magnetic field vector (T)
  Eigen::Vector3d localMagFieldVector_;

  // MAG simulation parameters
  MagSimParams magSimParams_;

  // MAG stochastic error state vectors (XYZ-axis)
  std::shared_ptr<StochasticErrors> pStochasticErrors_;

  // MAG constant errors
  Eigen::Vector3d hardIronBias_;
  Eigen::Matrix<double, 3, 3> softIronDistortion_;

  // Flag indicating if fixed local magnetic field vector is used
  bool useFixedLocalMagneticFieldVector_;

  // Model enable settings (enable/disable stochastic errors, etc.)
  ModelEnableSettings modelEnableSettings_;

  // MAG sample time (s)
  double dt_;

  // Random number generator for normal distribution
  std::mt19937 randomNumberGenerator_;
  unsigned int seed_;
  std::normal_distribution<> normalDistribution_;

  // Flag indicating if fixed random numbers are be used (for testing/debugging)
  bool useFixedRandomNumbers_;

  // Ideal measurement model (no errors, no distortion, etc.)
  Eigen::Vector3d calcIdealMeasurement(const Eigen::Quaterniond q_b_n_true);

  // Hard iron bias model
  Eigen::Vector3d calcHardIronBiasModel(const Eigen::Vector3d& measurement);

  // Soft iron distortion model
  Eigen::Vector3d calcSoftIronDistortionModel(
      const Eigen::Vector3d& measurement);

  // Stochastic error model (colored noise)
  Eigen::Vector3d calcStochasticErrorModel(const Eigen::Vector3d& measurement);

  // Update the stochastic error state space model (Gauß-Markov process)
  double updateDiscreteTimeEquivalentStateSpaceModel(
      std::shared_ptr<StochasticErrors> pStochasticErrors, const double N,
      const double B, const double K, const double corrTime,
      const AxisIdentifier axisId);

  // Saturation model
  Eigen::Vector3d calcSaturationModel(const Eigen::Vector3d& measurement);

  // Quantization model
  Eigen::Vector3d calcQuantizationModel(const Eigen::Vector3d& measurement);

  // Random number generation function
  double drawRandNormalDistNum();
};

}  // namespace mag_simulator
