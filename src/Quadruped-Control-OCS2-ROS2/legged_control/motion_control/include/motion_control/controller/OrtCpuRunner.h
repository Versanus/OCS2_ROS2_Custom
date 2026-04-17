#pragma once

#include <memory>
#include <string>
#include <vector>

#include "motion_control/controller/PolicyRunner.h"
#include "onnxruntime_cxx_api.h"

class OrtCpuRunner final : public PolicyRunner {
 public:
  OrtCpuRunner();
  ~OrtCpuRunner() override;

  bool load(const std::string& modelPath,
            const std::string& inputName,
            const std::string& outputName) override;
  bool infer(const std::vector<float>& observation, std::vector<float>& action) override;

  const std::string& inputName() const override;
  const std::string& outputName() const override;
  std::size_t inputSize() const override;
  std::size_t outputSize() const override;

 private:
  struct NamedTensorInfo {
    std::string name;
    std::size_t elementCount = 0;
    std::vector<int64_t> shape;
  };

  static NamedTensorInfo getTensorInfo(Ort::Session& session, bool isInput, std::size_t index);

  Ort::Env env_;
  Ort::SessionOptions sessionOptions_;
  std::unique_ptr<Ort::Session> session_;

  NamedTensorInfo inputInfo_;
  NamedTensorInfo outputInfo_;
  std::vector<int64_t> inputTensorShape_;
  std::vector<int64_t> outputTensorShape_;
};
