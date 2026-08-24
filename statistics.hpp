#ifndef statistics_HPP
#define statistics_HPP
#include <vector>

#include "boids.hpp"

namespace pf {
double mean_distance(std::vector<Boid> const& boid, double x_min, double x_max,
                     double y_min, double y_max);
double std_dev_distance(std::vector<Boid> const& boid, double const& mean,
                        double x_min, double x_max, double y_min, double y_max);
double mean_velocity(std::vector<Boid> const& boid);
double std_dev_velocity(std::vector<Boid> const& boid, double const& mean);
void print(std::vector<Boid> const& boid, double x_min, double x_max,
           double y_min, double y_max);

}  // namespace pf

#endif