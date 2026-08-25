#include "statistics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace pf {

double mean_distance(std::vector<Boid> const& boid, double x_min, double x_max,
                     double y_min, double y_max) {  // distanza media tra boids
  assert(!boid.empty());
  int n = static_cast<int>(boid.size());
  if (n == 1) {
    return 0.0;
  }
  double sum{0.0};
  for (int i = 0; i != n; ++i) {
    for (int j = i + 1; j != n; j++) {
      auto const i_sz = static_cast<std::size_t>(i);
      auto const j_sz = static_cast<std::size_t>(j);
      sum += std::sqrt(toroidal_distance_squared(boid[i_sz].pos, boid[j_sz].pos,
                                                 x_min, x_max, y_min, y_max));
    }
  }
  double const n_pairs = n * (n - 1.0) / 2.0;
  return sum * (1.0 / n_pairs);
}

// disperione che mi dice quando sono divere tra loro le distanze tra coppie,
// vogliamo farla rispetto al centro i massa?
double std_dev_distance(std::vector<Boid> const& boid, double const& mean,
                        double x_min, double x_max, double y_min,
                        double y_max) {
  assert(!boid.empty());
  int n = static_cast<int>(boid.size());
  if (n < 2) {
    return 0.0;
  }
  // double const mean = mean_distance(boid, x_min, x_max, y_min, y_max);
  double sum{0.0};
  for (int i = 0; i != n; ++i) {
    for (int j = i + 1; j != n; j++) {
      auto const i_sz = static_cast<std::size_t>(i);
      auto const j_sz = static_cast<std::size_t>(j);
      double const distance_ij = std::sqrt(toroidal_distance_squared(
          boid[i_sz].pos, boid[j_sz].pos, x_min, x_max, y_min, y_max));
      double const difference = distance_ij - mean;
      sum += difference * difference;
    }
  }
  double const n_pairs = n * (n - 1.0) / 2.0;
  return std::sqrt(sum / n_pairs);
}

double mean_velocity(std::vector<Boid> const& boid) {
  assert(!boid.empty());
  double sum{};
  int n = static_cast<int>(boid.size());
  if (n == 1) {
    return speed_modulus(boid[0].vel);
  }
  for (int i = 0; i != n; ++i) {
    sum += speed_modulus(boid[static_cast<std::size_t>(i)].vel);
  }
  return sum * (1.0 / n);
}

double std_dev_velocity(std::vector<Boid> const& boid, double const& mean) {
  assert(!boid.empty());
  int n = static_cast<int>(boid.size());
  if (n == 1) {
    return 0.0;
  }
  double sum{0.0};
  for (Boid const& m : boid) {
    double const speed = speed_modulus(m.vel);
    double const difference = speed - mean;
    sum += difference * difference;
  }
  return std::sqrt(sum / n);
}
void print(std::vector<Boid> const& boid, double x_min, double x_max,
           double y_min, double y_max) {
  double mean_distance_ = mean_distance(boid, x_min, x_max, y_min, y_max);
  double std_dev_distance_ =
      std_dev_distance(boid, mean_distance_, x_min, x_max, y_min, y_max);
  std::cout << "Distanza media: " << mean_distance_ << " +/- "
            << std_dev_distance_ << '\n';
  double mean_velocity_ = mean_velocity(boid);
  double std_dev_velocity_ = std_dev_velocity(boid, mean_velocity_);
  std::cout << "Velocità media:" << mean_velocity_ << " +/- "
            << std_dev_velocity_ << "\n \n \n";
}
}  // namespace pf
