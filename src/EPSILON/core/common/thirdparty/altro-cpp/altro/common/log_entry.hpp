// Copyright [2021] Optimus Ride Inc.

#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <string>

namespace altro {

// Simple replacement for fmt::color
enum class TextColor {
  white = 0,
  green,
  red,
  yellow,
  blue,
  cyan
};

/**
 * @brief Verbose output level
 */
enum class LogLevel {
  kSilent = 0,
  kOuter = 1,
  kOuterDebug = 2,
  kInner = 3,
  kInnerDebug = 4,
  kDebug = 5,
};

/**
 * @brief Represents an entry in the solver logger
 */
class LogEntry {
 public:
  enum EntryType { kInt, kFloat, kString };

  LogEntry() = default;
  explicit LogEntry(std::string title, std::string fmt, const EntryType& type = kFloat)
      : title_(title), name_(std::move(title)), format_(std::move(fmt)), type_(type) {}

  const std::string& GetFormat() const { return format_; }
  const std::string& GetTitle() const { return title_; }
  int GetWidth() const { return width_; }
  LogLevel GetLevel() const { return level_; }
  bool IsActive(const LogLevel level) { return level >= level_; }
  EntryType GetType() const { return type_; }

  LogEntry& SetLowerBound(const double& lb, TextColor color = TextColor::green);
  LogEntry& SetUpperBound(const double& ub, TextColor color = TextColor::red);
  LogEntry& SetWidth(const int& width);
  LogEntry& SetLevel(const LogLevel& level);
  LogEntry& SetType(const EntryType& type);
  LogEntry& SetName(const std::string& name);

  template <class T>
  void Log(T value) {
    color_ = GetColor(value);
    std::ostringstream oss;
    oss << std::setprecision(6) << value;
    data_ = oss.str();
  }

  void Print(const LogLevel level = LogLevel::kSilent) {
    if (IsActive(level)) {
      std::cout << std::setw(width_) << std::right << data_;
    }
  }

  void PrintHeader(const LogLevel level = LogLevel::kSilent, const TextColor color = TextColor::white) {
    if (IsActive(level)) {
      std::cout << std::setw(width_) << std::right << title_;
    }
  }

  template <class T>
  void Print(T value) {
    Log(value);
    Print();
  }

  void Clear() { data_.clear(); }

 private:
  template <class T>
  TextColor GetColor(T value) {
    TextColor color = color_default_;
    if (bounded_) {
      if (value < lower_) {
        color = color_lower_;
      } else if (value > upper_) {
        color = color_upper_;
      }
    }
    return color;
  }

  static constexpr int kDefaultWidth = 10;

  std::string title_;
  std::string name_;
  std::string format_;
  std::string data_;
  EntryType type_;
  LogLevel level_ = LogLevel::kInner;
  int width_ = kDefaultWidth;
  bool bounded_ = false;
  double lower_ = -std::numeric_limits<double>::infinity();
  double upper_ = +std::numeric_limits<double>::infinity();
  TextColor color_ = TextColor::white;
  TextColor color_default_ = TextColor::white;
  TextColor color_lower_ = TextColor::green;
  TextColor color_upper_ = TextColor::red;
};

}  // namespace altro
