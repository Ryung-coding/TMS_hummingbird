#pragma once
#include <cmath>
#include <params.hpp>
namespace utils {
struct LPF {
  double y = 0.0;
  double alpha = 1.0;
  bool initialized = false;
  explicit LPF(double convergence_time_sec = 0.0)
  {
    if (convergence_time_sec <= 0.0) { alpha = 1.0; return; }
    const double sample_count = convergence_time_sec * static_cast<double>(params::RATE_HZ);
    alpha = 1.0 - std::pow(0.01, 1.0 / sample_count);
  }
  double update(double x)
  {
    if (!initialized) { y = x; initialized = true; return y; }
    y = alpha * x + (1.0 - alpha) * y;
    return y;
  }
  void reset(double x = 0.0) { y = x; initialized = true; }
};
}
