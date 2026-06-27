#pragma once

#include <array>
#include <cstddef>

namespace openarm_cartesian_control {

constexpr size_t kCartesianImpedanceDoF = 6;

struct ImpedanceGains {
  std::array<double, kCartesianImpedanceDoF> k{};
  std::array<double, kCartesianImpedanceDoF> d{};
};

}  // namespace openarm_cartesian_control
