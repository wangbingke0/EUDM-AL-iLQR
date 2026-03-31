// Copyright [2021] Optimus Ride Inc.

#include "altro/common/timer.hpp"
#include "altro/common/profile_entry.hpp"
#include "altro/utils/assert.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace altro {

Timer::~Timer() { 
  if (!printed_summary_ && active_) {
    PrintSummary(); 
  }
  // Close the file if the Timer has ownship
  if (using_file_) {
    fclose(io_);
  }
}

void Timer::PrintSummary() {
  PrintSummary(&times_);
}

void Timer::PrintSummary(std::map<std::string, std::chrono::microseconds>* const times_ptr) {
  std::map<std::string, std::chrono::microseconds>& times = *times_ptr;

  using EntryPtr = std::shared_ptr<ProfileEntry>;
  std::vector<EntryPtr> entries;
  std::vector<EntryPtr> parents;

  // Add first (top level) entry
  auto times_iter = times.begin();
  entries.emplace_back(std::make_shared<ProfileEntry>("top", std::chrono::microseconds(0)));
  parents.emplace_back(entries.front());

  int max_width = 0;
  for (; times_iter != times.end(); ++times_iter) {
    entries.emplace_back(std::make_shared<ProfileEntry>(times_iter->first, times_iter->second));
    EntryPtr entry = entries.back();

    size_t curlevel = entry->NumLevels(); 

    if (curlevel >= parents.size()) {
      parents.resize(curlevel + 1);
    }

    parents[curlevel] = entry;
    entry->parent = parents[curlevel - 1];

    if (curlevel == 1) {
      entries.front()->time += entry->time;
    }

    int name_width = 0;
    for (const std::string& part : entry->name) {
      name_width += part.length();
    }
    max_width = std::max(max_width, name_width);
  }

  // Print the header
  const int pad = 2;
  max_width += pad;
  
  std::ostringstream header_oss;
  header_oss << std::left << std::setw(max_width) << "Description"
             << "  " << std::right << std::setw(8) << "Time (us)"
             << "  " << std::setw(7) << "%Total"
             << "  " << std::setw(7) << "%Parent";
  std::string header = header_oss.str();
  
  fprintf(io_, "%s\n", header.c_str());
  fprintf(io_, "%s\n", std::string(header.length(), '-').c_str());
  
  for (size_t i = 1; i < entries.size(); ++i) {
    entries[i]->CalcStats();
    entries[i]->Print(io_, max_width);
  }
  printed_summary_ = true;
}

Stopwatch Timer::Start(const std::string& name) {
  if (IsActive()) {
    stack_.emplace_back(name);
    std::string fullname = stack_.front();
    for (size_t i = 1; i < stack_.size(); ++i) {
      fullname += "/" + stack_[i];
    }
    return Stopwatch(std::move(fullname), shared_from_this());
  }
  return Stopwatch();
}

void Timer::SetOutput(const std::string& filename) {
  FILE* io = fopen(filename.c_str(), "w");
  if (io == nullptr) {
    std::ostringstream oss;
    oss << "Error opening profiler file \"" << filename << "\". Got errno " << errno << ".";
    throw std::runtime_error(oss.str());
  }
  SetOutput(io);

  // Set a flag that ensures the file will be closed when the Timer is destroyed
  using_file_ = true;
}

Stopwatch::Stopwatch(std::string name, std::shared_ptr<Timer> timer) 
    : name_(std::move(name)), 
      start_(std::chrono::high_resolution_clock::now()), 
      parent_(std::move(timer)) {}

Stopwatch::~Stopwatch() {
  if (parent_) {  
    auto duration = std::chrono::high_resolution_clock::now() - start_;
    microseconds us = std::chrono::duration_cast<microseconds>(duration);
    parent_->stack_.pop_back();
    parent_->times_[name_] += us;
  }
}


}  // namespace altro
