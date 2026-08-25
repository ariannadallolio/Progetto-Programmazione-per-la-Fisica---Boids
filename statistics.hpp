#ifndef statistics_HPP
#define statistics_HPP
#include <vector>

#include "boids.hpp"

namespace pf {
double mean_distance(std::vector<Boid> const& boid, Space const& space);
double std_dev_distance(std::vector<Boid> const& boid, double const& mean,
                        Space const& space);
double mean_velocity(std::vector<Boid> const& boid);
double std_dev_velocity(std::vector<Boid> const& boid, double const& mean);
void print(std::vector<Boid> const& boid, Space const& space);

}  // namespace pf

#endif