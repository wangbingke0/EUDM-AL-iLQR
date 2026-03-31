// Copyright [2021] Optimus Ride Inc.

#include "altro/common/solver_logger.hpp"

#include <iostream>
#include <iomanip>

namespace altro {


void SolverLogger::PrintHeader() {
  if (cur_level_ == LogLevel::kSilent) {
    return;
  }
  int total_width = 0;
  bool any_active = false;
  for (const std::string* title : order_) {
    LogEntry& entry = entries_[*title];
    entry.PrintHeader(cur_level_, header_color_);
    if (entry.IsActive(cur_level_)) {
      any_active = true;
      std::cout << " ";
      total_width += entry.GetWidth() + 1;
    }
  }
  if (any_active) {
    std::cout << "\n";
    std::cout << std::string(total_width, '-') << "\n";
  }
}

void SolverLogger::PrintData() {
  if (cur_level_ == LogLevel::kSilent) {
    return;
  }
  bool any_active = false;
  for (const std::string* title : order_) {
    LogEntry& entry = entries_[*title];
    entry.Print(cur_level_);
    if (entry.IsActive(cur_level_)) {
      any_active = true;
      std::cout << " ";
    }
  }
  if (any_active) {
    std::cout << "\n";
  }
}

void SolverLogger::Print() {
  if ((count_ % frequency_) == 0) {
    count_ = 0;
    PrintHeader();
  }
  PrintData();
  count_++;
}


void SolverLogger::Clear() {
  count_ = 0;
  for (auto& kv : entries_) {
    kv.second.Clear();
  }
}

}  // namespace altro
