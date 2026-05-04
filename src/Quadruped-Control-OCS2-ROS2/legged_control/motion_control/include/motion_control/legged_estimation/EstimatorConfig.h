#pragma once

#include <string>

enum class EstimatorType {
  Linear,
  InEKF,
};

enum class OrientationSource {
  ImuQuaternion,
  Ekf,
};

struct EstimatorConfig {
  bool enabled = true;
  EstimatorType type = EstimatorType::Linear;
  OrientationSource orientationSource = OrientationSource::ImuQuaternion;
};

EstimatorConfig loadEstimatorConfig(const std::string& configFile, bool enabledDefault);

const char* toString(EstimatorType estimatorType);
const char* toString(OrientationSource orientationSource);
