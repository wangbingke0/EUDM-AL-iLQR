// Copyright [2021] Optimus Ride Inc.

#include "altro/constraints/constraint.hpp"

#include <iostream>
#include <sstream>

namespace altro {
namespace constraints {

std::string ConstraintInfo::ToString(int precision) const {
  Eigen::IOFormat format(precision, 0, ", ", "", "", "", "[", "]");
  std::ostringstream oss;
  oss << label << " at index " << index << ": " << violation.format(format);
  return oss.str();
}

std::ostream& operator<<(std::ostream& os, const ConstraintInfo& coninfo) {
  return os << coninfo.ToString();
}

}  // namespace constraints
} // namespace altro