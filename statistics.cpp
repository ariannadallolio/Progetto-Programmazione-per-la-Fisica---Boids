#include "statistics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>

namespace pf {

Statistics statistics(std::vector<Boid> const& boid, Space const& space) {
  assert(!boid.empty());

  int n = static_cast<int>(boid.size());

  if (n < 2) {
    return {0.0, 0.0, speed_modulus(boid[0].vel), 0.0};
  }

  double const n_pairs = n * (n - 1.0) / 2.0;

  std::vector<double>
      distances;
  distances.reserve(static_cast<std::size_t>(n_pairs));
  for (int i = 0; i != n; ++i) {
    for (int j = i + 1; j != n; j++) {
      auto const i_sz = static_cast<std::size_t>(i);
      auto const j_sz = static_cast<std::size_t>(j);
      double const distance_ij = std::sqrt(
          toroidal_distance_squared(boid[i_sz].pos, boid[j_sz].pos, space));
      distances.push_back(distance_ij);
    }
  }
  double mean_distance_sum =
      std::accumulate(distances.begin(), distances.end(), 0.0);
  double const mean_distance = mean_distance_sum * (1.0 / n_pairs);

  double std_dev_distance_sum{0.0};
  for (double d : distances) {
    double const difference = d - mean_distance;
    std_dev_distance_sum += difference * difference;
  }
  double const std_dev_distance = sqrt(std_dev_distance_sum / n_pairs);

  
  
  std::vector<double>
      speeds;
  speeds.reserve(boid.size());
  for (int i = 0; i != n; ++i) {
    double modulus = speed_modulus(boid[static_cast<std::size_t>(i)].vel);
    speeds.push_back(modulus);
  }
  double mean_velocity_sum = std::accumulate(speeds.begin(), speeds.end(), 0.0);
  double const mean_velocity = mean_velocity_sum / n;

  double std_dev_velocity_sum{0.0};
  for (double s : speeds) {
    double const difference = s - mean_velocity;
    std_dev_velocity_sum += difference * difference;
  }
  double const std_dev_velocity = sqrt(std_dev_velocity_sum / n);

  return {mean_distance, std_dev_distance, mean_velocity, std_dev_velocity};
}

void print(Statistics const& stats, int seconds) {
  std::cout << seconds << " s" << '\n'
            << "Mean distance: " << stats.mean_distance << " +/- "
            << stats.std_dev_distance << '\n'
            << "Mean Velocity:" << stats.mean_velocity << " +/- "
            << stats.std_dev_velocity << "\n \n \n";
}
void save_for_root(Statistics const& stats, std::ofstream& file,
                   int frame_count) {
  file << frame_count << '\t' << stats.mean_distance << '\t'
       << stats.std_dev_distance << '\t' << stats.mean_velocity << '\t'
       << stats.std_dev_velocity << '\n';
}
}  // namespace pf
