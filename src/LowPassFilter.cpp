#include "LowPassFilter.h"

#include <cmath>
#include <iostream>

LowPassFilter::LowPassFilter(
    double sampling_rate,
    double cutoff_frequency,
    int dof,
    std::atomic<std::uint64_t>* nan_counter)
  : nan_counter_(nan_counter)
{
  // Initialize alpha (filter coefficient) based on sampling rate and cutoff
  // frequency and set previous torque vector size to dof
  alpha_ = 2.0*M_PI*cutoff_frequency/(2.0*M_PI*cutoff_frequency + sampling_rate);
  tau_J_prev.resize(dof);
  filtered_tau_J.resize(dof);
  std::cout << "Low pass filter initialized with alpha = " << alpha_
            << std::endl;
}

void LowPassFilter::initLPF(std::vector<double>& init_tau_J)
{
  // Initialize the filter with the first measurement
  tau_J_prev = init_tau_J;
}

std::vector<double> LowPassFilter::getFilteredEffort(
    std::vector<double>& tau_J_raw)
{
  for (std::size_t i = 0; i < filtered_tau_J.size(); i++)
  {
    filtered_tau_J[i] =
        tau_J_prev[i] * alpha_ + tau_J_raw[i] * (1.0 - alpha_);

    // NaN-trap recovery.  Historically `tau_J_prev[i]` could latch
    // non-finite — any one bad input contaminated every subsequent
    // sample and only a process restart cleared it.  Force the
    // stored previous value back to a finite number on every cycle:
    //   * feed-through `tau_J_raw[i]` if it's finite (the new
    //     measurement is good — believe it, just skip the filter
    //     this sample)
    //   * else fall back to zero (both sides are non-finite — at
    //     least produce something the downstream consumer can use)
    if (!std::isfinite(filtered_tau_J[i]))
    {
      filtered_tau_J[i] = std::isfinite(tau_J_raw[i]) ? tau_J_raw[i] : 0.0;
      if (nan_counter_ != nullptr)
        nan_counter_->fetch_add(1, std::memory_order_relaxed);
    }

    tau_J_prev[i] = filtered_tau_J[i];
  }
  return filtered_tau_J;
}
