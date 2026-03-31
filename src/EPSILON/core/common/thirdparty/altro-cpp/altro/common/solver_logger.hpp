// Copyright [2021] Optimus Ride Inc.

#pragma once

#include <iostream>
#include <sstream>
#include <limits>
#include <vector>
#include <map>
#include <unordered_map>

#include "altro/utils/assert.hpp"
#include "altro/common/log_entry.hpp"

namespace altro {

/**
 * @brief Provides a tabular-like logging output
 */
class SolverLogger {
 public:
  explicit SolverLogger(const LogLevel level = LogLevel::kSilent) : cur_level_(level) {}

  LogLevel GetLevel() const { return cur_level_; }
  LogEntry& GetEntry(const std::string& title) { return entries_[title]; }
  int NumEntries() { return entries_.size(); }

  /*************************** Iteration **************************************/
  using iterator = std::unordered_map<std::string, LogEntry>::iterator;
  using const_iterator = std::unordered_map<std::string, LogEntry>::const_iterator;
  iterator begin() { return entries_.begin(); }
  const_iterator begin() const { return entries_.cbegin(); }
  iterator end() { return entries_.end(); }
  const_iterator end() const { return entries_.cend(); }

  template <class... Args>
  LogEntry& AddEntry(const int& col, Args... args);

  void SetLevel(const LogLevel level) { cur_level_ = level; }
  void Disable() { SetLevel(LogLevel::kSilent); }

  void SetFrequency(const int freq) {
    ALTRO_ASSERT(freq >= 0, "Header print frequency must be positive.");
    frequency_ = freq;
  }

  template <class T>
  void Log(const std::string& title, T value);

  void PrintHeader();
  void PrintData();
  void Print();
  void Clear();

  void SetHeaderColor(const TextColor color) { header_color_ = color; }

 private:
  static constexpr int kDefaultFrequency = 10;

  LogLevel cur_level_ = LogLevel::kSilent;
  int frequency_ = kDefaultFrequency;
  int count_ = 0;
  std::unordered_map<std::string, LogEntry> entries_;
  std::vector<const std::string*> order_;
  TextColor header_color_ = TextColor::white;
};

template <class... Args>
LogEntry& SolverLogger::AddEntry(const int& col, Args... args) {
  int num_entries = static_cast<int>(entries_.size());
  ALTRO_ASSERT(
      col <= num_entries,
      "Column must be less than or equal to the current number of entries.");
  ALTRO_ASSERT(
      col >= -num_entries - 1,
      "Column must be greater or equal to than the negative new number of entries.");

  LogEntry entry(std::forward<Args>(args)...);
  const std::string title = entry.GetTitle();
  auto insert = entries_.emplace(std::make_pair(title, std::move(entry)));

  const std::string* title_ptr = &(insert.first->first);

  auto it = order_.begin();
  if (col < 0) {
    it = order_.end() + col + 1;
  } else {
    it += col;
  }
  order_.insert(it, title_ptr);

  return insert.first->second;
}

template <class T>
void SolverLogger::Log(const std::string& title, T value) {
  if (cur_level_ > LogLevel::kSilent && entries_[title].IsActive(cur_level_)) {
    entries_[title].Log(value);
  }
}

}  // namespace altro
