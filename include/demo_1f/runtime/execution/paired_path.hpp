#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace demo_1f {
using JointPath = std::vector<std::vector<double>>;

// Cumulative joint-space (Euclidean) length at each point of a path.
inline std::vector<double> path_lengths(const JointPath & path) {
  std::vector<double> lengths{0.0};
  for (size_t i = 1; i < path.size(); ++i) {
    double step = 0;
    for (size_t j = 0; j < path[i].size(); ++j) {
      step += std::pow(path[i][j] - path[i - 1][j], 2);
    }
    lengths.push_back(lengths.back() + std::sqrt(step));
  }
  return lengths;
}

// Point of a path at progress s (0..1 of its length), linear between its points.
inline std::vector<double> path_at(
    const JointPath & path, const std::vector<double> & lengths, double s) {
  const double total = lengths.back();
  if (total <= 0 || s <= 0) return path.front();
  if (s >= 1) return path.back();
  const double reach = s * total;
  for (size_t i = 1; i < path.size(); ++i) {
    if (lengths[i] < reach || lengths[i] <= lengths[i - 1]) continue;
    const double t = (reach - lengths[i - 1]) / (lengths[i] - lengths[i - 1]);
    std::vector<double> point(path[i].size());
    for (size_t j = 0; j < point.size(); ++j) {
      point[j] = path[i - 1][j] + t * (path[i][j] - path[i - 1][j]);
    }
    return point;
  }
  return path.back();
}

// Rows of first's then second's joints at every corner of either path, by normalized length.
inline JointPath pair_paths(const JointPath & first, const JointPath & second) {
  for (const auto * path : {&first, &second}) {
    if (path->empty() || path->front().empty()) {
      throw std::invalid_argument("Paired paths need at least one point with joints");
    }
    for (const auto & point : *path) {
      if (point.size() != path->front().size()) {
        throw std::invalid_argument("Paired path points differ in joint count");
      }
    }
  }
  const auto first_lengths = path_lengths(first), second_lengths = path_lengths(second);
  std::vector<double> progress{0.0, 1.0};
  for (const auto * lengths : {&first_lengths, &second_lengths}) {
    if (lengths->back() <= 0) continue;
    for (double length : *lengths) progress.push_back(length / lengths->back());
  }
  std::sort(progress.begin(), progress.end());
  JointPath rows;
  double previous = -1;
  for (double s : progress) {
    if (s - previous < 1e-9) continue;
    previous = s;
    auto row = path_at(first, first_lengths, s);
    const auto other = path_at(second, second_lengths, s);
    row.insert(row.end(), other.begin(), other.end());
    rows.push_back(std::move(row));
  }
  return rows;
}
} // namespace demo_1f
