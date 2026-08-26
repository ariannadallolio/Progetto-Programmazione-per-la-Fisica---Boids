#include "statistics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace pf {

Statistics statistics(std::vector<Boid> const& boid, Space const& space) {
  assert(!boid.empty());

  int n = static_cast<int>(boid.size());

  if (n < 2) {
    return {0.0, 0.0, speed_modulus(boid[0].vel), 0.0};
  }

  double const n_pairs = n * (n - 1.0) / 2.0;
  std::vector<double>
      distances;  // vettore temporaneo per salvare le distanze e usarle
                  // per la deviazione standard, così da calcolarle una
                  // sola volta e ottimizzare la funzione
  double mean_distance_sum{0.0};
  for (int i = 0; i != n; ++i) {
    for (int j = i + 1; j != n; j++) {
      auto const i_sz = static_cast<std::size_t>(i);
      auto const j_sz = static_cast<std::size_t>(j);
      double const distance_ij = std::sqrt(
          toroidal_distance_squared(boid[i_sz].pos, boid[j_sz].pos, space));
      distances.push_back(distance_ij);
      mean_distance_sum += distance_ij;
    }
  }
  double const mean_distance = mean_distance_sum * (1.0 / n_pairs);

  double std_dev_distance_sum{0.0};
  for (double d : distances) {
    double const difference = d - mean_distance;
    std_dev_distance_sum += difference * difference;
  }
  double const std_dev_distance = sqrt(std_dev_distance_sum / n_pairs);

  std::vector<double>
      speeds;  // vettore temporaneo per salvare le velocità e usarle
               // per la deviazione standard, così da calcolarle una
               // sola volta e ottimizzare la funzione
  double mean_velocity_sum{0.0};
  for (int i = 0; i != n; ++i) {
    double modulus = speed_modulus(boid[static_cast<std::size_t>(i)].vel);
    speeds.push_back(modulus);
    mean_velocity_sum += modulus;
  }
  double const mean_velocity = mean_velocity_sum / n;

  double std_dev_velocity_sum{0.0};
  for (double s : speeds) {
    double const difference = s - mean_velocity;
    std_dev_velocity_sum += difference * difference;
  }
  double const std_dev_velocity = sqrt(std_dev_velocity_sum / n);

  return {mean_distance, std_dev_distance, mean_velocity, std_dev_velocity};
}


void print(std::vector<Boid> const& boid, Space const& space) {
  Statistics stats = statistics(boid, space);
  std::cout << "Mean distance: " << stats.mean_distance << " +/- "
            << stats.std_dev_distance << '\n'
            << "Mean Velocity:" << stats.mean_velocity << " +/- "
            << stats.std_dev_velocity << "\n \n \n";
}
}  // namespace pf
