#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <atomic>
#include <cstdint>
#include <vector>

// First-order IIR low-pass filter for joint torque feedback and (in
// current-control mode) torque command.
//
// **NaN safety** — historical bug: a single non-finite input would
// pollute `tau_J_prev` and every subsequent output stayed NaN until
// the LowPassFilter was re-constructed (i.e. process restart).  The
// `getFilteredEffort` implementation now detects non-finite output
// and forces `tau_J_prev` back to a finite value (feed-through raw
// if it's finite, else zero), breaking the sticky-NaN trap.  When a
// non-null `nan_counter` is supplied to the constructor, each reset
// increments it — useful for the kortex_hardware diagnostics
// pipeline.  Pass nullptr (the default) when counting isn't needed.
class LowPassFilter
{
public:
  LowPassFilter(double sampling_rate,
                double cutoff_frequency,
                int dof,
                std::atomic<std::uint64_t>* nan_counter = nullptr);

  void initLPF(std::vector<double>& init_tau_J);

  std::vector<double> getFilteredEffort(std::vector<double>& tau_J_raw);

private:
  double alpha_;
  std::vector<double> tau_J_prev;
  std::vector<double> filtered_tau_J;
  std::atomic<std::uint64_t>* nan_counter_;
};

#endif
