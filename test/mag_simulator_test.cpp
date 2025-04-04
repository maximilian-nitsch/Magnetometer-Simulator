/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "mag_simulator.h"

#include "gtest/gtest.h"

/**
 * @brief Test fixture for the MAG simulator class.
*/
class MagSimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Simulation parameters, identified with for PNI RM3100 magnetometer
    mag_simulator::MagSimParams magSimParams;

    magSimParams.N =
        Eigen::Vector3d(0.0107403325174045 * 1e-06, 0.0155945417657915 * 1e-06,
                        0.0149346442616531 * 1e-06);  // (T/s^(1/2))
    magSimParams.B = Eigen::Vector3d(0.00273246827281338 * 1e-06,
                                     0.00184704594648358 * 1e-06,
                                     0.0018816647975664 * 1e-06);  // (T/s)
    magSimParams.K = Eigen::Vector3d(
        8.30181561710663e-05 * 1e-06, 5.88363096771479e-05 * 1e-06,
        7.70561623631383e-05 * 1e-06);  // (T/s^(3/2))
    magSimParams.corrTime = Eigen::Vector3d(79.6349908634337, 497.649977057122,
                                            85.2398861596715);          // (s)
    magSimParams.hardIronBias = Eigen::Vector3d(0.0, 0.0, 0.0);         // (T)
    magSimParams.softIronDistortion = Eigen::Matrix3d::Identity();      // (-)
    magSimParams.measRange = Eigen::Vector3d(8e-04, 8e-04, 8e-04);      // (T)
    magSimParams.resolution = Eigen::Vector3d(13e-09, 13e-09, 13e-09);  // (T)
    magSimParams.sensorRotation = Eigen::Matrix3d::Identity();          // (-)

    // Model enable settings
    mag_simulator::ModelEnableSettings modelEnableSettings;

    modelEnableSettings.enableHardIronBias = false;
    modelEnableSettings.enableSoftIronDistortion = false;
    modelEnableSettings.enableStochasticError = false;
    modelEnableSettings.enableSaturation = false;
    modelEnableSettings.enableQuantization = false;

    // MAG sample time
    double sampleTime = 0.01;

    // Random number generator seed
    unsigned int seed = 42;

    // Local magnetic field vector (T) at Neumayer III station
    Eigen::Vector3d localMagFieldVector =
        Eigen::Vector3d(17.98356571986 * 1e-06, -4.75537810022174 * 1e-06,
                        -32.8983169172881 * 1e-06);

    // Initialize the MAG simulator class
    magSimulator =
        mag_simulator::MagSimulator(localMagFieldVector, magSimParams,
                                    modelEnableSettings, sampleTime, seed);
  }

  // Declare the class under test
  mag_simulator::MagSimulator magSimulator;
};

/**
 * @brief Test setter and getter function for the local magnetic field vector.
*/
TEST_F(MagSimulatorTest, LocalMagFieldVectorSetAndGetTest) {
  // Get local magnetic field vector
  Eigen::Vector3d m_n = magSimulator.getLocalMagFieldVector();
  Eigen::Vector3d m_n_expected =
      Eigen::Vector3d(17.98356571986 * 1e-06, -4.75537810022174 * 1e-06,
                      -32.8983169172881 * 1e-06);

  // Check if the parameters are set correctly in constructor
  EXPECT_NEAR(m_n(0), m_n_expected(0), 1e-10);
  EXPECT_NEAR(m_n(1), m_n_expected(1), 1e-10);
  EXPECT_NEAR(m_n(2), m_n_expected(2), 1e-10);

  // Set new local magnetic field vector
  m_n_expected = Eigen::Vector3d(19762e-09, 970e-09, 44963e-09);
  magSimulator.setLocalMagFieldVector(m_n_expected);
  m_n = magSimulator.getLocalMagFieldVector();

  // Check if the parameters are set correctly
  EXPECT_NEAR(m_n(0), m_n_expected(0), 1e-10);
  EXPECT_NEAR(m_n(1), m_n_expected(1), 1e-10);
  EXPECT_NEAR(m_n(2), m_n_expected(2), 1e-10);
}

/**
 * @brief Test setter and getter function for the magnetometer simulator parameters.
*/
TEST_F(MagSimulatorTest, MagSimParamsSetAndGetTest) {
  //  Test parameters for the magnetometer simulator
  mag_simulator::MagSimParams magSimParamsExpected;

  magSimParamsExpected.N = Eigen::Vector3d(1.0, 2.0, 3.0);  // (T/s^(1/2))
  magSimParamsExpected.B = Eigen::Vector3d(4.0, 5.0, 6.0);  // (T/s)
  magSimParamsExpected.K = Eigen::Vector3d(7.0, 8.0, 9.0);  // (T/s^(3/2))
  magSimParamsExpected.corrTime = Eigen::Vector3d(10.0, 11.0, 12.0);      // (s)
  magSimParamsExpected.hardIronBias = Eigen::Vector3d(13.0, 14.0, 15.0);  // (T)
  magSimParamsExpected.softIronDistortion << 16.0, 17.0, 18.0, 19.0, 20.0, 21.0,
      22.0, 23.0, 24.0;                                                 // (-)
  magSimParamsExpected.measRange = Eigen::Vector3d(25.0, 26.0, 27.0);   // (T)
  magSimParamsExpected.resolution = Eigen::Vector3d(28.0, 29.0, 30.0);  // (T)
  magSimParamsExpected.sensorRotation << 0, -1, 0, 1, 0, 0, 0, 0, -1;   // (-)

  // Set new magnetometer simulator parameters
  magSimulator.setMagSimParams(magSimParamsExpected);
  mag_simulator::MagSimParams magSimParams = magSimulator.getMagSimParams();

  // Check if the parameters are set correctly
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(magSimParams.N(i), magSimParamsExpected.N(i), 1e-10);
    EXPECT_NEAR(magSimParams.B(i), magSimParamsExpected.B(i), 1e-10);
    EXPECT_NEAR(magSimParams.K(i), magSimParamsExpected.K(i), 1e-10);
    EXPECT_NEAR(magSimParams.corrTime(i), magSimParamsExpected.corrTime(i),
                1e-10);
    EXPECT_NEAR(magSimParams.hardIronBias(i),
                magSimParamsExpected.hardIronBias(i), 1e-10);

    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(magSimParams.softIronDistortion(i, j),
                  magSimParamsExpected.softIronDistortion(i, j), 1e-10);
      EXPECT_NEAR(magSimParams.sensorRotation(i, j),
                  magSimParamsExpected.sensorRotation(i, j), 1e-10);
    }

    EXPECT_NEAR(magSimParams.measRange(i), magSimParamsExpected.measRange(i),
                1e-10);
    EXPECT_NEAR(magSimParams.resolution(i), magSimParamsExpected.resolution(i),
                1e-10);
  }
}

/**
 * @brief Test setter and getter function for the random number generator seed.
*/
TEST_F(MagSimulatorTest, SimulatorSeedSetAndGetTest) {
  // Set new random number generator seed
  magSimulator.setSimulatorSeed(43);
  // Get random number generator seed
  unsigned int seed = magSimulator.getSimulatorSeed();

  // Check if the parameters are set correctly
  EXPECT_EQ(seed, 43);
}

/**
 * @brief Test setter and getter function for the sample time.
*/
TEST_F(MagSimulatorTest, SampleTimeSetAndGetTest) {
  // Set new sample time
  magSimulator.setMagSampleTime(0.02);

  // Get sample time
  double sampleTime = magSimulator.getMagSampleTime();

  // Check if the parameters are set correctly
  EXPECT_NEAR(sampleTime, 0.02, 1e-10);
}

/**
 * @brief Test setter and getter function for the model enable settings.
*/
TEST_F(MagSimulatorTest, ModelEnableSettingsSetAndGetTest) {
  //  Test parameters for the magnetic field simulator
  mag_simulator::ModelEnableSettings modelEnableSettings;

  // Define new model enable settings
  modelEnableSettings.enableHardIronBias = true;
  modelEnableSettings.enableSoftIronDistortion = true;
  modelEnableSettings.enableStochasticError = true;
  modelEnableSettings.enableSaturation = true;
  modelEnableSettings.enableQuantization = true;

  // Set new model enable settings
  magSimulator.setModelEnableSettings(modelEnableSettings);

  // Get model enable settings
  mag_simulator::ModelEnableSettings modelEnableSettingsFromSimulator =
      magSimulator.getModelEnableSettings();

  // Check if the parameters are set correctly
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableHardIronBias, true);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableSoftIronDistortion, true);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableStochasticError, true);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableSaturation, true);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableQuantization, true);

  // Define new model enable settings
  modelEnableSettings.enableHardIronBias = false;
  modelEnableSettings.enableSoftIronDistortion = false;
  modelEnableSettings.enableStochasticError = false;
  modelEnableSettings.enableSaturation = false;
  modelEnableSettings.enableQuantization = false;

  // Set new model enable settings
  magSimulator.setModelEnableSettings(modelEnableSettings);

  // Get model enable settings
  modelEnableSettingsFromSimulator = magSimulator.getModelEnableSettings();

  // Check if the parameters are set correctly
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableHardIronBias, false);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableSoftIronDistortion, false);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableStochasticError, false);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableSaturation, false);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableQuantization, false);

  // Set using individual setter functions
  magSimulator.setEnableHardIronBias(true);
  magSimulator.setEnableSoftIronDistortion(true);
  magSimulator.setEnableStochasticError(true);
  magSimulator.setEnableSaturation(true);
  magSimulator.setEnableQuantization(true);

  // Get model enable settings
  modelEnableSettingsFromSimulator = magSimulator.getModelEnableSettings();

  // Check if the parameters are set correctly
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableHardIronBias, true);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableSoftIronDistortion, true);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableStochasticError, true);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableSaturation, true);
  EXPECT_EQ(modelEnableSettingsFromSimulator.enableQuantization, true);
}

/**
 * @brief Test setter and getter function for using fixed random numbers flag.
*/
TEST_F(MagSimulatorTest, UseFixedRandomNumbersFlagSetAndGetTest) {
  // Set using fixed random numbers flag
  magSimulator.setUseFixedRandomNumbersFlag(true);

  // Get using fixed random numbers flag
  bool useFixedRandomNumbersFlag = magSimulator.getUseFixedRandomNumbersFlag();

  // Check if the parameter is set correctly
  EXPECT_EQ(useFixedRandomNumbersFlag, true);

  // Set using fixed random numbers flag
  magSimulator.setUseFixedRandomNumbersFlag(false);

  // Get using fixed random numbers flag
  useFixedRandomNumbersFlag = magSimulator.getUseFixedRandomNumbersFlag();

  // Check if the parameter is set correctly
  EXPECT_EQ(useFixedRandomNumbersFlag, false);
}

/** 
 * @brief Test the print function for the magnetometer simulator parameters.
*/
TEST_F(MagSimulatorTest, PrintMagSimulatorParametersTest) {
  // Print the magnetometer simulator parameters
  std::stringstream ss = magSimulator.printMagSimulatorParameters();
  // Check if output is not empty
  EXPECT_FALSE(ss.str().empty());
}

/**
 * @brief Test the ideal measurement model.
*/
TEST_F(MagSimulatorTest, IdealMeasurementTest) {
  // Define the true attitude quaternion
  Eigen::Quaterniond q_b_n_true(1.0, 0.0, 0.0, 0.0);

  // Define the sensor rotation matrix
  Eigen::Matrix3d C_b_s = Eigen::Matrix3d::Identity();

  // Get local magnetic field vector
  Eigen::Vector3d m_n = magSimulator.getLocalMagFieldVector();

  // Generate an ideal measurement with the simulator
  Eigen::Vector3d m_s_meas =
      magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Calculate the expected measurement
  Eigen::Vector3d m_s_expected =
      C_b_s * q_b_n_true.toRotationMatrix().transpose() * m_n;

  // Check if the ideal measurement is calculated correctly
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);

  // Change the local magnetic field vector
  m_n = Eigen::Vector3d(20000e-09, 10000e-09, 50000e-09);

  // Set new local magnetic field vector
  magSimulator.setLocalMagFieldVector(m_n);

  // Change the sensor rotation matrix
  C_b_s << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0;

  // Get model parameters and change the sensor rotation matrix
  mag_simulator::MagSimParams magSimParams = magSimulator.getMagSimParams();
  magSimParams.sensorRotation = C_b_s;

  // Set new model parameters
  magSimulator.setMagSimParams(magSimParams);

  // Generate an ideal measurement with the simulator
  m_s_meas = magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Calculate the expected measurement
  m_s_expected = Eigen::Vector3d(10000e-09, -20000e-09, -50000e-09);

  // Check if the ideal measurement is calculated correctly
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);

  // Change the true attitude quaternion (roll=90deg, pitch=0deg, yaw=180deg)
  q_b_n_true = Eigen::Quaterniond(4.32978028117747e-17, 0.707106781186548,
                                  0.707106781186547, 4.32978028117747e-17);

  // Change the sensor rotation matrix
  magSimParams.sensorRotation = Eigen::Matrix3d::Identity();

  // Set new model parameters
  magSimulator.setMagSimParams(magSimParams);

  // Generate an ideal measurement with the simulator
  m_s_meas = magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Calculate the expected measurement
  m_s_expected = Eigen::Vector3d(10000e-09, 20000e-09, -50000e-09);

  // Check if the ideal measurement is calculated correctly
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);
}

/**
 * @brief Test the soft iron distortion model.
*/
TEST_F(MagSimulatorTest, SoftIronDistortioModelTest) {
  // Define the local magnetic field vector
  Eigen::Vector3d m_n = Eigen::Vector3d(20000e-09, 10000e-09, 50000e-09);

  // Set new local magnetic field vector
  magSimulator.setLocalMagFieldVector(m_n);

  // Get magnetometer simulation parameters
  mag_simulator::MagSimParams magSimParams = magSimulator.getMagSimParams();

  // Define the soft iron distortion matrix
  magSimParams.softIronDistortion << 1.1, -2.2, 3.3, -4.4, 5.5, -6.6, 7.7, -8.8,
      9.9;

  // Set new magnetometer simulation parameters
  magSimulator.setMagSimParams(magSimParams);

  // Enable soft iron distortion error model
  magSimulator.setEnableSoftIronDistortion(true);

  // Define the true attitude quaternion
  Eigen::Quaterniond q_b_n_true(1.0, 0.0, 0.0, 0.0);

  // Define the sensor rotation matrix
  Eigen::Matrix3d C_b_s = Eigen::Matrix3d::Identity();

  // Generate a measurement with the simulator
  Eigen::Vector3d m_s_meas =
      magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Calculate the expected measurement
  Eigen::Vector3d m_s_expected = magSimParams.softIronDistortion * C_b_s *
                                 q_b_n_true.toRotationMatrix().transpose() *
                                 m_n;

  // Check if the measurement with hard iron bias error is calculated correctly
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);
}

/**
 * @brief Test the hard iron bias model.
*/
TEST_F(MagSimulatorTest, HardIronBiasModelTest) {
  // Define the local magnetic field vector
  Eigen::Vector3d m_n = Eigen::Vector3d(20000e-09, 10000e-09, 50000e-09);

  // Set new local magnetic field vector
  magSimulator.setLocalMagFieldVector(m_n);

  // Get magnetometer simulation parameters
  mag_simulator::MagSimParams magSimParams = magSimulator.getMagSimParams();

  // Define the hard iron bias
  magSimParams.hardIronBias = Eigen::Vector3d(1000e-09, -2000e-09, 5000e-09);

  // Set new magnetometer simulation parameters
  magSimulator.setMagSimParams(magSimParams);

  // Enable hard iron bias error model
  magSimulator.setEnableHardIronBias(true);

  // Define the true attitude quaternion
  Eigen::Quaterniond q_b_n_true(1.0, 0.0, 0.0, 0.0);

  // Define the sensor rotation matrix
  Eigen::Matrix3d C_b_s = Eigen::Matrix3d::Identity();

  // Generate a measurement with the simulator
  Eigen::Vector3d m_s_meas =
      magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Calculate the expected measurement
  Eigen::Vector3d m_s_expected =
      C_b_s * q_b_n_true.toRotationMatrix().transpose() * m_n +
      magSimParams.hardIronBias;

  // Check if the measurement with hard iron bias error is calculated correctly
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);
}

/**
 * @brief Test if stochastic errors are zero when noise parameters are zero.
*/
TEST_F(MagSimulatorTest, StochasticErrorModelZeroNoiseTest) {
  // Define the local magnetic field vector
  Eigen::Vector3d m_n = Eigen::Vector3d(20000e-09, 10000e-09, 50000e-09);

  // Set new local magnetic field vector
  magSimulator.setLocalMagFieldVector(m_n);

  // Get magnetometer simulation parameters
  mag_simulator::MagSimParams magSimParams = magSimulator.getMagSimParams();

  // Define the stochastic error parameters
  magSimParams.N = Eigen::Vector3d(0.0, 0.0, 0.0);               // (T/s^(1/2))
  magSimParams.B = Eigen::Vector3d(0.0, 0.0, 0.0);               // (T/s)
  magSimParams.K = Eigen::Vector3d(0.0, 0.0, 0.0);               // (T/s^(3/2))
  magSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);  // (s)

  // Set new magnetometer simulation parameters
  magSimulator.setMagSimParams(magSimParams);

  // Enable stochastic error model
  magSimulator.setEnableStochasticError(true);

  // Define the true attitude quaternion
  Eigen::Quaterniond q_b_n_true(1.0, 0.0, 0.0, 0.0);

  // Define the sensor rotation matrix
  Eigen::Matrix3d C_b_s = Eigen::Matrix3d::Identity();

  // Generate a measurement with the simulator
  Eigen::Vector3d m_s_meas =
      magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Calculate the expected measurement which is the ideal measurement
  Eigen::Vector3d m_s_expected =
      C_b_s * q_b_n_true.toRotationMatrix().transpose() * m_n;

  // Magnetometer measurement should be the ideal measurement
  EXPECT_EQ(m_s_meas(0), m_s_expected(0));
  EXPECT_EQ(m_s_meas(1), m_s_expected(1));
  EXPECT_EQ(m_s_meas(2), m_s_expected(2));

  // Get the stochastic errors
  mag_simulator::StochasticErrors stochasticErrors =
      magSimulator.getStochasticErrors();

  // Stochastic errors should be zero
  EXPECT_EQ(stochasticErrors.z_N(0), 0.0);
  EXPECT_EQ(stochasticErrors.z_N(1), 0.0);
  EXPECT_EQ(stochasticErrors.z_N(2), 0.0);

  EXPECT_EQ(stochasticErrors.z_B(0), 0.0);
  EXPECT_EQ(stochasticErrors.z_B(1), 0.0);
  EXPECT_EQ(stochasticErrors.z_B(2), 0.0);

  EXPECT_EQ(stochasticErrors.z_K(0), 0.0);
  EXPECT_EQ(stochasticErrors.z_K(1), 0.0);
  EXPECT_EQ(stochasticErrors.z_K(2), 0.0);
}

/**
 * @brief Test the stochastic error model with fixed random numbers.
*/
TEST_F(MagSimulatorTest, StochasticErrorModelTest) {
  // Define the local magnetic field vector
  Eigen::Vector3d m_n = Eigen::Vector3d(20000e-09, 10000e-09, 50000e-09);

  // Set new local magnetic field vector
  magSimulator.setLocalMagFieldVector(m_n);

  // Get magnetometer simulation parameters
  mag_simulator::MagSimParams magSimParams = magSimulator.getMagSimParams();

  // Define the stochastic error parameters
  magSimParams.N = Eigen::Vector3d(0.001, 0.001, 0.001);         // (T/s^(1/2))
  magSimParams.B = Eigen::Vector3d(0.0001, 0.0001, 0.0001);      // (T/s)
  magSimParams.K = Eigen::Vector3d(1e-5, 1e-5, 1e-5);            // (T/s^(3/2))
  magSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);  // (s)

  // Set new magnetometer simulation parameters
  magSimulator.setMagSimParams(magSimParams);

  // Enable stochastic error model
  magSimulator.setEnableStochasticError(true);

  // Set the random numbers to fixed values for testing
  magSimulator.setUseFixedRandomNumbersFlag(true);

  // Define the true attitude quaternion
  Eigen::Quaterniond q_b_n_true(1.0, 0.0, 0.0, 0.0);

  // Define the sensor rotation matrix
  Eigen::Matrix3d C_b_s = Eigen::Matrix3d::Identity();

  // Generate a measurement with the simulator
  Eigen::Vector3d m_s_meas =
      magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Get the simulated stochastic errors
  mag_simulator::StochasticErrors stochasticErrors =
      magSimulator.getStochasticErrors();

  // Expected stochastic errors
  Eigen::Vector3d z_N_expected = Eigen::Vector3d(0.01, 0.01, 0.01);
  Eigen::Vector3d z_B_expected = Eigen::Vector3d(
      3.04352466221447e-06, 3.04352466221447e-06, 3.04352466221447e-06);
  Eigen::Vector3d z_K_expected = Eigen::Vector3d(3e-06, 3e-06, 3e-06);

  // Calculate the expected measurement
  Eigen::Vector3d m_s_expected =
      C_b_s * q_b_n_true.toRotationMatrix().transpose() * m_n + z_N_expected;

  // Check if the measurement is as expected for fixed random numbers
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);

  // Check if the stochastic errors are as expected for fixed random numbers
  EXPECT_NEAR(stochasticErrors.z_N(0), z_N_expected(0), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_N(1), z_N_expected(1), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_N(2), z_N_expected(2), 1e-10);

  EXPECT_NEAR(stochasticErrors.z_B(0), z_B_expected(0), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_B(1), z_B_expected(1), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_B(2), z_B_expected(2), 1e-10);

  EXPECT_NEAR(stochasticErrors.z_K(0), z_K_expected(0), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_K(1), z_K_expected(1), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_K(2), z_K_expected(2), 1e-10);

  // Generate second measurement with the simulator
  m_s_meas = magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Check that the stochastic errors changed after second measurement
  stochasticErrors = magSimulator.getStochasticErrors();

  EXPECT_FALSE(stochasticErrors.z_N.isApprox(z_N_expected, 1e-10));
  EXPECT_FALSE(stochasticErrors.z_B.isApprox(z_B_expected, 1e-10));
  EXPECT_FALSE(stochasticErrors.z_K.isApprox(z_K_expected, 1e-10));

  // Expected stochastic errors after second measurement
  z_N_expected = Eigen::Vector3d(0.0100060435246622, 0.0100060435246622,
                                 0.0100060435246622);
  z_B_expected = Eigen::Vector3d(6.08674498717983e-06, 6.08674498717983e-06,
                                 6.08674498717983e-06);
  z_K_expected = Eigen::Vector3d(6e-06, 6e-06, 6e-06);

  // Calculate the expected second measurement
  m_s_expected =
      C_b_s * q_b_n_true.toRotationMatrix().transpose() * m_n + z_N_expected;

  // Check if the second measurement is as expected for fixed random numbers
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);

  // Check if the stochastic errors are as expected for fixed random numbers
  EXPECT_NEAR(stochasticErrors.z_N(0), z_N_expected(0), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_N(1), z_N_expected(1), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_N(2), z_N_expected(2), 1e-10);

  EXPECT_NEAR(stochasticErrors.z_B(0), z_B_expected(0), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_B(1), z_B_expected(1), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_B(2), z_B_expected(2), 1e-10);

  EXPECT_NEAR(stochasticErrors.z_K(0), z_K_expected(0), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_K(1), z_K_expected(1), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_K(2), z_K_expected(2), 1e-10);

  // Reset the simulator
  magSimulator.resetSimulator();

  // Check that the stochastic errors are zero after reset
  stochasticErrors = magSimulator.getStochasticErrors();

  EXPECT_EQ(stochasticErrors.z_N(0), 0.0);
  EXPECT_EQ(stochasticErrors.z_N(1), 0.0);
  EXPECT_EQ(stochasticErrors.z_N(2), 0.0);

  EXPECT_EQ(stochasticErrors.z_B(0), 0.0);
  EXPECT_EQ(stochasticErrors.z_B(1), 0.0);
  EXPECT_EQ(stochasticErrors.z_B(2), 0.0);

  EXPECT_EQ(stochasticErrors.z_K(0), 0.0);
  EXPECT_EQ(stochasticErrors.z_K(1), 0.0);
  EXPECT_EQ(stochasticErrors.z_K(2), 0.0);

  // Generate third measurement with the simulator
  m_s_meas = magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Check that the stochastic are as for the first measurement
  stochasticErrors = magSimulator.getStochasticErrors();

  // Expected stochastic errors
  z_N_expected = Eigen::Vector3d(0.01, 0.01, 0.01);
  z_B_expected = Eigen::Vector3d(3.04352466221447e-06, 3.04352466221447e-06,
                                 3.04352466221447e-06);
  z_K_expected = Eigen::Vector3d(3e-06, 3e-06, 3e-06);

  // Calculate the expected measurement
  m_s_expected =
      C_b_s * q_b_n_true.toRotationMatrix().transpose() * m_n + z_N_expected;

  // Check if the measurement is as expected for fixed random numbers
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);

  // Check if the stochastic errors are as expected for fixed random numbers
  EXPECT_NEAR(stochasticErrors.z_N(0), z_N_expected(0), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_N(1), z_N_expected(1), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_N(2), z_N_expected(2), 1e-10);

  EXPECT_NEAR(stochasticErrors.z_B(0), z_B_expected(0), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_B(1), z_B_expected(1), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_B(2), z_B_expected(2), 1e-10);

  EXPECT_NEAR(stochasticErrors.z_K(0), z_K_expected(0), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_K(1), z_K_expected(1), 1e-10);
  EXPECT_NEAR(stochasticErrors.z_K(2), z_K_expected(2), 1e-10);
}

/**
 * @brief Test the saturation model.
*/
TEST_F(MagSimulatorTest, SaturationModelTest) {
  // Define the local magnetic field vector
  Eigen::Vector3d m_n = Eigen::Vector3d(20000e-09, -10000e-09, 50000e-09);

  // Set new local magnetic field vector
  magSimulator.setLocalMagFieldVector(m_n);

  // Get magnetometer simulation parameters
  mag_simulator::MagSimParams magSimParams = magSimulator.getMagSimParams();

  // Define the saturation parameters
  magSimParams.measRange = Eigen::Vector3d(1000e-09, 1000e-09, 1000e-09);

  // Set new magnetometer simulation parameters
  magSimulator.setMagSimParams(magSimParams);

  // Enable saturation model
  magSimulator.setEnableSaturation(true);

  // Define the true attitude quaternion
  Eigen::Quaterniond q_b_n_true(1.0, 0.0, 0.0, 0.0);

  // Generate a measurement with the simulator
  Eigen::Vector3d m_s_meas =
      magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Calculate the expected measurement
  Eigen::Vector3d m_s_expected = Eigen::Vector3d(1000e-09, -1000e-09, 1000e-09);

  // Check if the measurement is saturated as expected
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);
}

/**
 * @brief Test the quantization model.
*/
TEST_F(MagSimulatorTest, QuantizationModelTest) {
  // Define the local magnetic field vector
  Eigen::Vector3d m_n = Eigen::Vector3d(20000e-09, -10000e-09, 50000e-09);

  // Set new local magnetic field vector
  magSimulator.setLocalMagFieldVector(m_n);

  // Get magnetometer simulation parameters
  mag_simulator::MagSimParams magSimParams = magSimulator.getMagSimParams();

  // Define the quantization parameters
  magSimParams.resolution = Eigen::Vector3d(30000e-09, 10000e-09, 12500e-09);

  // Set new magnetometer simulation parameters
  magSimulator.setMagSimParams(magSimParams);

  // Enable quantization model
  magSimulator.setEnableQuantization(true);

  // Define the true attitude quaternion
  Eigen::Quaterniond q_b_n_true(1.0, 0.0, 0.0, 0.0);

  // Generate a measurement with the simulator
  Eigen::Vector3d m_s_meas =
      magSimulator.generateMagnetometerMeasurement(q_b_n_true);

  // Calculate the expected measurement
  Eigen::Vector3d m_s_expected =
      Eigen::Vector3d(30000e-09, -10000e-09, 50000e-09);

  // Check if the measurement is quantized as expected
  EXPECT_NEAR(m_s_meas(0), m_s_expected(0), 1e-10);
  EXPECT_NEAR(m_s_meas(1), m_s_expected(1), 1e-10);
  EXPECT_NEAR(m_s_meas(2), m_s_expected(2), 1e-10);
}
