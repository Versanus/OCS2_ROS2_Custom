#pragma once

#include <cstddef>
#include <string>
#include <vector>

class PolicyRunner {
 public:
  virtual ~PolicyRunner() = default;

  virtual bool load(const std::string& modelPath,
                    const std::string& inputName,
                    const std::string& outputName) = 0;
  virtual bool infer(const std::vector<float>& observation, std::vector<float>& action) = 0;

  virtual const std::string& inputName() const = 0;
  virtual const std::string& outputName() const = 0;
  virtual std::size_t inputSize() const = 0;
  virtual std::size_t outputSize() const = 0;
};
