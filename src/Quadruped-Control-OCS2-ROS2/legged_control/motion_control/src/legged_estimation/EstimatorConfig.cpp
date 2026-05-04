#include "motion_control/legged_estimation/EstimatorConfig.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace {

std::string normalizeToken(std::string value) {
  value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char ch) {
                return ch == '_' || ch == '-' || std::isspace(ch);
              }),
              value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

EstimatorType parseEstimatorType(const std::string& value) {
  const std::string token = normalizeToken(value);
  if (token.empty() || token == "linear" || token == "linearkalman" || token == "kalman") {
    return EstimatorType::Linear;
  }
  if (token == "inekf" || token == "invariantekf" || token == "invariantekf") {
    return EstimatorType::InEKF;
  }

  throw std::runtime_error("stateEstimator.type must be 'linear' or 'inekf'.");
}

OrientationSource parseOrientationSource(const std::string& value) {
  const std::string token = normalizeToken(value);
  if (token.empty() || token == "imu" || token == "imuquat" || token == "imuquaternion" || token == "quaternion") {
    return OrientationSource::ImuQuaternion;
  }
  if (token == "ekf" || token == "estimate" || token == "estimated") {
    return OrientationSource::Ekf;
  }

  throw std::runtime_error("stateEstimator.orientationSource must be 'imu_quaternion' or 'ekf'.");
}

}  // namespace

EstimatorConfig loadEstimatorConfig(const std::string& configFile, bool enabledDefault) {
  boost::property_tree::ptree tree;
  boost::property_tree::read_info(configFile, tree);

  EstimatorConfig config;
  config.enabled = tree.get<bool>("stateEstimator.enabled", enabledDefault);
  config.type = parseEstimatorType(tree.get<std::string>("stateEstimator.type", "linear"));
  config.orientationSource =
      parseOrientationSource(tree.get<std::string>("stateEstimator.orientationSource", "imu_quaternion"));
  return config;
}

const char* toString(EstimatorType estimatorType) {
  switch (estimatorType) {
    case EstimatorType::Linear:
      return "linear";
    case EstimatorType::InEKF:
      return "inekf";
    default:
      return "unknown";
  }
}

const char* toString(OrientationSource orientationSource) {
  switch (orientationSource) {
    case OrientationSource::ImuQuaternion:
      return "imu_quaternion";
    case OrientationSource::Ekf:
      return "ekf";
    default:
      return "unknown";
  }
}
