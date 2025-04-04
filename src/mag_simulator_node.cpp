/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "nanoauv_sensor_driver_interfaces/msg/magnetometer.hpp"

#include "mag_simulator.h"

namespace mag_simulator {

/**
 * @brief MAG simulator node class.
 * 
 * This class is the main node class of the MAG simulator package. It is
 * responsible for the initialization of the MAG simulator object, the
 * subscription to the ground truth odometry topic, and the publication of the
 * simulated MAG data and diagnostic messages.
 * 
*/
class MagSimulatorNode : public rclcpp::Node {
 public:
  // Constructor
  explicit MagSimulatorNode(std::shared_ptr<MagSimulator> pMagSimulator);

  // Destructor
  ~MagSimulatorNode() {}

 private:
  // MAG simulator class object
  std::shared_ptr<MagSimulator> pMagSimulator_;

  // Ground truth odometry message
  nav_msgs::msg::Odometry::SharedPtr groundTruthOdomMsg_;

  // Last odometry timestamp
  rclcpp::Time lastOdomTimestamp_;

  // MAG message publisher
  rclcpp::Publisher<nanoauv_sensor_driver_interfaces::msg::Magnetometer>::
      SharedPtr pCustomMagPublisher_;

  // Vehicle odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pOdometrySubscriber_;

  // Timers
  rclcpp::TimerBase::SharedPtr pTimer_;
  rclcpp::TimerBase::SharedPtr pOdometryTimeOutTimer_;

  // Loop counter
  unsigned int count_;

  // Sample time
  double sampleTime_;

  // Odometry flags
  bool first_odometry_received_;
  bool odometry_timeout_;

  // Node namespace
  std::string node_namespace_;

  // Decleration and retrieval functions for parameters from YAML file
  void declareAndRetrieveGeneralSettings();
  void declareAndRetrieveMagParameters();
  void declareAndRetrieveEnableSettings();
  void declareAndRetrieveEnvironmentalSettings();

  // Timer callback function
  void magSimulatorLoopCallback();

  // Odometry callback functions
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometryTimeOutCallback();

  // Helper functions
  Eigen::Vector3d doubleVectorToEigenVector3(const std::vector<double>& vec);
};

/**
 * @brief MAG simulator node constructor with default config file path.
 * 
 * @param[in] pMagSimulator Pointer to the MAG simulator object
*/
MagSimulatorNode::MagSimulatorNode(std::shared_ptr<MagSimulator> pMagSimulator)
    : Node("mag_simulator_node"),
      pMagSimulator_(pMagSimulator),
      lastOdomTimestamp_(0, 0),
      count_(0),
      sampleTime_(0.0),
      first_odometry_received_(false),
      odometry_timeout_(false),
      node_namespace_(this->get_namespace()) {
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Configuring MAG simulator node...");

  // Declare adnd retrieve parameters and load them to MAG simulator
  declareAndRetrieveGeneralSettings();
  declareAndRetrieveMagParameters();
  declareAndRetrieveEnableSettings();
  declareAndRetrieveEnvironmentalSettings();

  // Reset MAG simulator to ensure that constant errors are initialized
  pMagSimulator_->resetSimulator();

  // Print MAG simulator parameters
  std::stringstream ss = pMagSimulator_->printMagSimulatorParameters();
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_), "%s", ss.str().c_str());

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Parameters from YAML config loaded successfully.");

  // Get the value of the topic_name parameter
  this->declare_parameter("topic_name", rclcpp::PARAMETER_STRING);

  // Get topic name from launch file or use default
  std::string groundTruthTopicName;
  this->get_parameter_or("topic_name", groundTruthTopicName,
                         std::string("/auv/odometry"));

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Subscribing to ground truth odometry topic: %s",
              groundTruthTopicName.c_str());

  // Initialize the odometry subscriber
  pOdometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      groundTruthTopicName, 10,
      std::bind(&MagSimulatorNode::odometryCallback, this,
                std::placeholders::_1));

  // Initialize the custom MAG data publisher
  pCustomMagPublisher_ = this->create_publisher<
      nanoauv_sensor_driver_interfaces::msg::Magnetometer>(
      node_namespace_ + "/magnetic_field", 10);

  // Extract MAG sample time from the parameter server
  double sampleTime =
      get_parameter("mag_simulator.general_settings.sample_time").as_double();

  // Set class member sample time
  sampleTime_ = sampleTime;

  // Convert sample time to milliseconds and cast to int
  int sampleTimeInt = static_cast<int>(sampleTime * 1e3);

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "MAG simulator node executing with %dms.", sampleTimeInt);

  // Create a timer to call the MAG simulator loop callback function
  pTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(sampleTimeInt),
      std::bind(&MagSimulatorNode::magSimulatorLoopCallback, this));

  // Create timer for timeout
  pOdometryTimeOutTimer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&MagSimulatorNode::odometryTimeOutCallback, this));

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "MAG simulator node initialized. Node waiting for first odometry "
              "message...");
}

/**
 * @brief Declare and retrieve general settings from the parameter server.
*/
void MagSimulatorNode::declareAndRetrieveGeneralSettings() {
  // General settings
  double sampleTime;
  int seed;
  bool useConstantSeed;

  // Declare general settings
  this->declare_parameter("mag_simulator.general_settings.sample_time",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mag_simulator.general_settings.seed",
                          rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("mag_simulator.general_settings.use_constant_seed",
                          rclcpp::PARAMETER_BOOL);

  // Retrieve general settings
  sampleTime = this->get_parameter("mag_simulator.general_settings.sample_time")
                   .as_double();
  seed = this->get_parameter("mag_simulator.general_settings.seed").as_int();
  useConstantSeed =
      this->get_parameter("mag_simulator.general_settings.use_constant_seed")
          .as_bool();

  // Set MAG simulator sample time and seed
  pMagSimulator_->setMagSampleTime(sampleTime);

  // Set seed depending on the useConstantSeed flag
  if (useConstantSeed == false) {
    // Draw a random seed from the random device
    std::random_device rd;
    seed = rd();
    pMagSimulator_->setSimulatorSeed(seed);
    RCLCPP_INFO(rclcpp::get_logger(node_namespace_), "Using random seed: %d",
                seed);
  } else {
    // Set the random number generator seed
    pMagSimulator_->setSimulatorSeed(seed);
    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "Using seed from config file: %d", seed);
  }
}

/**
 * @brief Declare and retrieve MAG model parameters from the parameter server.
*/
void MagSimulatorNode::declareAndRetrieveMagParameters() {
  // Magnetometer model parameters
  MagSimParams magSimParams;

  // Declare model parameters
  this->declare_parameter("mag_simulator.model_parameter_settings.N",
                          rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("mag_simulator.model_parameter_settings.B",
                          rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("mag_simulator.model_parameter_settings.K",
                          rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "mag_simulator.model_parameter_settings.correlation_time",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "mag_simulator.model_parameter_settings.soft_iron_distortion",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "mag_simulator.model_parameter_settings.hard_iron_bias",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "mag_simulator.model_parameter_settings.measurement_range",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("mag_simulator.model_parameter_settings.resolution",
                          rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "mag_simulator.model_parameter_settings.sensor_rotation",
      rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Retrieve model parameters
  std::vector<double> N =
      this->get_parameter("mag_simulator.model_parameter_settings.N")
          .as_double_array();
  std::vector<double> B =
      this->get_parameter("mag_simulator.model_parameter_settings.B")
          .as_double_array();
  std::vector<double> K =
      this->get_parameter("mag_simulator.model_parameter_settings.K")
          .as_double_array();
  std::vector<double> corrTime =
      this->get_parameter(
              "mag_simulator.model_parameter_settings.correlation_time")
          .as_double_array();
  std::vector<double> softIronDistortion =
      this->get_parameter(
              "mag_simulator.model_parameter_settings.soft_iron_distortion")
          .as_double_array();
  std::vector<double> hardIronBias =
      this->get_parameter(
              "mag_simulator.model_parameter_settings.hard_iron_bias")
          .as_double_array();
  std::vector<double> measRange =
      this->get_parameter(
              "mag_simulator.model_parameter_settings.measurement_range")
          .as_double_array();
  std::vector<double> resolution =
      this->get_parameter("mag_simulator.model_parameter_settings.resolution")
          .as_double_array();
  std::vector<double> sensorRotation =
      this->get_parameter(
              "mag_simulator.model_parameter_settings.sensor_rotation")
          .as_double_array();

  // Assign model parameters to struct
  magSimParams.N = doubleVectorToEigenVector3(N);
  magSimParams.B = doubleVectorToEigenVector3(B);
  magSimParams.K = doubleVectorToEigenVector3(K);
  magSimParams.corrTime = doubleVectorToEigenVector3(corrTime);
  magSimParams.hardIronBias = doubleVectorToEigenVector3(hardIronBias);
  magSimParams.softIronDistortion << softIronDistortion[0],
      softIronDistortion[1], softIronDistortion[2], softIronDistortion[3],
      softIronDistortion[4], softIronDistortion[5], softIronDistortion[6],
      softIronDistortion[7], softIronDistortion[8];
  magSimParams.measRange = doubleVectorToEigenVector3(measRange);
  magSimParams.resolution = doubleVectorToEigenVector3(resolution);

  // Extract sensor rotation as Euler angles in ZYX order from the parameter server
  Eigen::Vector3d sensorRotationEulerRpy =
      doubleVectorToEigenVector3(sensorRotation) * M_PI / 180.0;  // deg to rad

  // Convert Euler angles to rotation matrix
  magSimParams.sensorRotation =
      Eigen::AngleAxisd(sensorRotationEulerRpy[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(sensorRotationEulerRpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(sensorRotationEulerRpy[2], Eigen::Vector3d::UnitX());

  // Set MAG simulator parameters
  pMagSimulator_->setMagSimParams(magSimParams);
}

/**
 * @brief Declare and retrieve MAG model enable settings from the parameter server.
*/
void MagSimulatorNode::declareAndRetrieveEnableSettings() {
  // Model enable settings
  ModelEnableSettings modelEnableSettings;

  // Model enable settings
  this->declare_parameter(
      "mag_simulator.model_enable_settings.enable_soft_iron_distortion",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "mag_simulator.model_enable_settings.enable_hard_iron_bias",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "mag_simulator.model_enable_settings.enable_stochastic_error",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "mag_simulator.model_enable_settings.enable_saturation",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "mag_simulator.model_enable_settings.enable_quantization",
      rclcpp::PARAMETER_BOOL);

  // Retrieve model enable settings
  bool enableSoftIronDistortion = this->get_parameter(
                                          "mag_simulator.model_enable_settings."
                                          "enable_soft_iron_distortion")
                                      .as_bool();
  bool enableHardIronBias = this->get_parameter(
                                    "mag_simulator.model_enable_settings."
                                    "enable_hard_iron_bias")
                                .as_bool();
  bool enableStochasticError = this->get_parameter(
                                       "mag_simulator.model_enable_settings."
                                       "enable_stochastic_error")
                                   .as_bool();
  bool enableSaturation = this->get_parameter(
                                  "mag_simulator.model_enable_settings."
                                  "enable_saturation")
                              .as_bool();
  bool enableQuantization = this->get_parameter(
                                    "mag_simulator.model_enable_settings."
                                    "enable_quantization")
                                .as_bool();

  // Assign model enable settings to struct
  modelEnableSettings.enableSoftIronDistortion = enableSoftIronDistortion;
  modelEnableSettings.enableHardIronBias = enableHardIronBias;
  modelEnableSettings.enableStochasticError = enableStochasticError;
  modelEnableSettings.enableSaturation = enableSaturation;
  modelEnableSettings.enableQuantization = enableQuantization;

  // Set MAG simulator enable settings
  pMagSimulator_->setModelEnableSettings(modelEnableSettings);
}

/**
 * @brief Declare and retrieve environmental settings from the parameter server.
*/
void MagSimulatorNode::declareAndRetrieveEnvironmentalSettings() {
  // Local magnetic field vector
  Eigen::Vector3d localMagneticFieldVector;

  // Declare environmental settings
  this->declare_parameter(
      "mag_simulator.environmental_settings.local_magnetic_field_vector",
      rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Retrieve environmental settings
  std::vector<double> localMagneticFieldVectorDouble =
      this->get_parameter(
              "mag_simulator.environmental_settings.local_magnetic_field_"
              "vector")
          .as_double_array();

  // Assign local magnetic field vector to Eigen vector
  localMagneticFieldVector =
      doubleVectorToEigenVector3(localMagneticFieldVectorDouble);

  // Set MAG simulator parameters
  pMagSimulator_->setLocalMagFieldVector(localMagneticFieldVector);
}

/**
 * @brief MAG simulator loop callback function.
 * 
 * This function is called periodically by the timer and publishes the MAG
 * and diagnostic messages. The MAG simulator is updated with ground truth
 * data and the simulated MAG data is published. 
*/
void MagSimulatorNode::magSimulatorLoopCallback() {
  // Get current timestamp
  rclcpp::Time currentTimestamp = now();

  // Create messages
  nanoauv_sensor_driver_interfaces::msg::Magnetometer customMagMsg;
  geometry_msgs::msg::Vector3 magMsg;
  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg;

  // Create the custom MAG message
  customMagMsg.header.stamp = currentTimestamp;
  customMagMsg.header.frame_id = "mag_link";

  // Set the custom IMU message data validity flag to false
  customMagMsg.is_valid.data = false;

  // Read out odometry message
  Eigen::Quaterniond q_ib_true;

  // Check if ground truth odometry message is available
  if (groundTruthOdomMsg_ == nullptr) {
    // Print STALE diagnostic message when no ground truth odometry message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "MAG Simulator";
    diagnosticMsg.message = "Waiting for first ground truth odometry message!";

    // Fill the custom MAG message with diagnostic status message
    customMagMsg.diagnostic_status = diagnosticMsg;

    // Publish the custom MAG message
    pCustomMagPublisher_->publish(customMagMsg);

    // Reset odometry timeout timer since waiting for first message
    pOdometryTimeOutTimer_->cancel();
    pOdometryTimeOutTimer_->reset();

    return;

  } else {
    // Assign ground truth odometry message to MAG simulator inputs
    q_ib_true.w() = groundTruthOdomMsg_.get()->pose.pose.orientation.w;
    q_ib_true.x() = groundTruthOdomMsg_.get()->pose.pose.orientation.x;
    q_ib_true.y() = groundTruthOdomMsg_.get()->pose.pose.orientation.y;
    q_ib_true.z() = groundTruthOdomMsg_.get()->pose.pose.orientation.z;

    // Generate magnetometer measurement
    Eigen::Vector3d m_n_meas =
        pMagSimulator_->generateMagnetometerMeasurement(q_ib_true);

    // Fill magnetic field message with simulated data
    magMsg.x = m_n_meas(0);
    magMsg.y = m_n_meas(1);
    magMsg.z = m_n_meas(2);

    // Fill the custom MAG message with simulated data
    customMagMsg.magnetic_field = magMsg;
    customMagMsg.is_valid.data = true;

    // // Fill magnetic field message with ideal data
    // Eigen::Matrix3d C_n_b = q_ib_true.toRotationMatrix().transpose();

    // Eigen::Vector3d m_n_ideal =
    //     C_n_b * pMagSimulator_->getLocalMagFieldVector();

    // magMsg.magnetic_field.x = m_n_ideal(0);
    // magMsg.magnetic_field.y = m_n_ideal(1);
    // magMsg.magnetic_field.z = m_n_ideal(2);

    // // Publish the ideal MAG message
    // pIdealMagPublisher_->publish(magMsg);

    // Fill the diagnostic message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnosticMsg.name = "MAG Simulator";
    diagnosticMsg.message = "MAG simulator running nominal.";
  }

  // Calculate time since last odometry message
  rclcpp::Duration timeSinceLastOdom =
      rclcpp::Duration(currentTimestamp - lastOdomTimestamp_);

  if (timeSinceLastOdom.seconds() > sampleTime_ && odometry_timeout_ == false) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnosticMsg.name = "MAG Simulator";
    diagnosticMsg.message =
        "Ground truth odometry message frequency is too slow!"
        " MAG simulator ground truth frequency higher than odometry!"
        " Increase odometry message frequency!";
  }

  if (odometry_timeout_ == true) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "MAG Simulator";
    diagnosticMsg.message =
        "No ground truth odometry message received since than 5 seconds!"
        " MAG simulator stalling!";
  }

  // Fill the custom MAG message with diagnostic status message
  customMagMsg.diagnostic_status = diagnosticMsg;

  // Publish the custom MAG message
  pCustomMagPublisher_->publish(customMagMsg);

  // Increase loop counter
  count_++;
}

/**
 * @brief Odometry callback function.
 * 
 * This function is called when a new odometry message is received. The ground
 * truth odometry message is assigned to the ground truth odometry message
 * member variable of the node class.
 * 
 * @param[in] msg Pointer to the odometry message
*/
void MagSimulatorNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Reset odometry timeout timer
  pOdometryTimeOutTimer_->cancel();
  pOdometryTimeOutTimer_->reset();

  // Set first odometry received flag
  if (first_odometry_received_ == false) {
    first_odometry_received_ = true;

    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "First ground truth odometry message received! MAG simulator "
                "now running nominal!");
  }
  // Reset odometry timeout flag
  if (odometry_timeout_ == true) {
    odometry_timeout_ = false;

    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "Ground truth odometry message received after timeout! MAG "
                "simulator now running nominal!");
  }

  // Assign ground truth odometry message
  groundTruthOdomMsg_ = msg;

  //   Assign last odometry timestamp
  lastOdomTimestamp_ = msg->header.stamp;
}

/**
 * @brief Odometry timeout callback function.
 * 
 * This function is called when no ground truth odometry message is received.
 * 
*/
void MagSimulatorNode::odometryTimeOutCallback() {
  // Set odometry timeout flag
  odometry_timeout_ = true;

  RCLCPP_WARN_THROTTLE(rclcpp::get_logger(node_namespace_), *get_clock(), 5000,
                       "No ground truth odometry message since more than 5 "
                       "seconds! MAG simulator now starting to stale!");
}

/** 
 * @brief Helper function to convert a vector of doubles to an Eigen vector.
 * 
 * @param[in] vec Vector of doubles
 * @return Eigen vector
*/
Eigen::Vector3d MagSimulatorNode::doubleVectorToEigenVector3(
    const std::vector<double>& vec) {
  // Convert vector of doubles to Eigen vector
  Eigen::Vector3d eigenVec;

  // Check vector size
  if (vec.size() != 3) {
    RCLCPP_ERROR(rclcpp::get_logger(node_namespace_),
                 "doubleVectorToEigenVector3: Vector size is not 3!");
  } else {
    eigenVec << vec[0], vec[1], vec[2];
  }

  return eigenVec;
}

}  // namespace mag_simulator

/**
 * @brief Main function of the MAG simulator node.
 * 
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 * 
 * @return int Return value
*/
int main(int argc, char** argv) {
  // Create MAG simulator object
  std::shared_ptr<mag_simulator::MagSimulator> pMagSimulator =
      std::make_shared<mag_simulator::MagSimulator>();

  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<mag_simulator::MagSimulatorNode>(pMagSimulator));
  rclcpp::shutdown();

  return 0;
}
