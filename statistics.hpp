#ifndef STATISTICS_HPP
#define STATISTICS_HPP
#include <fstream>
#include <vector>

#include "boids.hpp"

namespace pf {

struct Statistics {
  double mean_distance;
  double std_dev_distance;
  double mean_velocity;
  double std_dev_velocity;
};

Statistics statistics(std::vector<Boid> const& boids, Space const& space);
void print(Statistics const& stats);
void save_for_root(Statistics const& stats, std::ofstream& file, double time);

}  // namespace pf

#endif