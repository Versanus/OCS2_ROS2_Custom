#include "motion_control/controller/OrtCpuRunner.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace {

std::vector<int64_t> normalizeTensorShape(const std::vector<int64_t>& shape) {
  std::vector<int64_t> normalized = shape;
  if (normalized.empty()) {
    normalized.push_back(1);
    return normalized;
  }

  for (auto& dim : normalized) {
    if (dim <= 0) {
      dim = 1;
    }
  }

  if (normalized.size() == 1) {
    normalized.insert(normalized.begin(), 1);
  }

  return normalized;
}

}  // namespace

OrtCpuRunner::OrtCpuRunner()
    : env_(ORT_LOGGING_LEVEL_WARNING, "legged_rl_cpu_runner"),
      sessionOptions_(),
      session_() {
  sessionOptions_.SetIntraOpNumThreads(1);
  sessionOptions_.SetInterOpNumThreads(1);
  sessionOptions_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
}

OrtCpuRunner::~OrtCpuRunner() = default;

bool OrtCpuRunner::load(const std::string& modelPath,
                        const std::string& inputName,
                        const std::string& outputName) {
  try {
    session_ = std::make_unique<Ort::Session>(env_, modelPath.c_str(), sessionOptions_);
    if (session_->GetInputCount() == 0 || session_->GetOutputCount() == 0) {
      throw std::runtime_error("Model must expose at least one input and one output tensor.");
    }

    inputInfo_ = getTensorInfo(*session_, true, 0);
    outputInfo_ = getTensorInfo(*session_, false, 0);

    if (!inputName.empty() && inputInfo_.name != inputName) {
      throw std::runtime_error("Configured input name '" + inputName + "' does not match model input '" + inputInfo_.name + "'.");
    }
    if (!outputName.empty() && outputInfo_.name != outputName) {
      throw std::runtime_error("Configured output name '" + outputName + "' does not match model output '" + outputInfo_.name + "'.");
    }

    inputTensorShape_ = normalizeTensorShape(inputInfo_.shape);
    outputTensorShape_ = normalizeTensorShape(outputInfo_.shape);
    return true;
  } catch (...) {
    session_.reset();
    inputInfo_ = NamedTensorInfo{};
    outputInfo_ = NamedTensorInfo{};
    inputTensorShape_.clear();
    outputTensorShape_.clear();
    return false;
  }
}

bool OrtCpuRunner::infer(const std::vector<float>& observation, std::vector<float>& action) {
  if (!session_) {
    return false;
  }
  if (observation.size() != inputInfo_.elementCount) {
    return false;
  }

  try {
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<float> inputBuffer = observation;
    auto inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, inputBuffer.data(), inputBuffer.size(), inputTensorShape_.data(), inputTensorShape_.size());

    const char* inputNames[] = {inputInfo_.name.c_str()};
    const char* outputNames[] = {outputInfo_.name.c_str()};
    auto outputValues = session_->Run(Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);
    if (outputValues.empty() || !outputValues.front().IsTensor()) {
      return false;
    }

    auto& outputTensor = outputValues.front();
    const auto tensorInfo = outputTensor.GetTensorTypeAndShapeInfo();
    const auto outputElementCount = tensorInfo.GetElementCount();
    const float* outputData = outputTensor.GetTensorData<float>();
    action.assign(outputData, outputData + outputElementCount);
    return action.size() == outputInfo_.elementCount;
  } catch (...) {
    return false;
  }
}

const std::string& OrtCpuRunner::inputName() const {
  return inputInfo_.name;
}

const std::string& OrtCpuRunner::outputName() const {
  return outputInfo_.name;
}

std::size_t OrtCpuRunner::inputSize() const {
  return inputInfo_.elementCount;
}

std::size_t OrtCpuRunner::outputSize() const {
  return outputInfo_.elementCount;
}

OrtCpuRunner::NamedTensorInfo OrtCpuRunner::getTensorInfo(Ort::Session& session, bool isInput, std::size_t index) {
  Ort::AllocatorWithDefaultOptions allocator;
  NamedTensorInfo info;

  Ort::AllocatedStringPtr name =
      isInput ? session.GetInputNameAllocated(index, allocator) : session.GetOutputNameAllocated(index, allocator);
  info.name = name.get();

  Ort::TypeInfo typeInfo = isInput ? session.GetInputTypeInfo(index) : session.GetOutputTypeInfo(index);
  auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
  info.shape = tensorInfo.GetShape();
  info.elementCount = tensorInfo.GetElementCount();
  return info;
}
