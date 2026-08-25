#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <random>
#include <vector>

#include "boids.hpp"

namespace pf {

struct Parameters {
  double s;
  double a;
  double c;
  double d;
  double d_s;
  double v_min;
  double v_max;
  double dt;
};

void check_parameters(Parameters const& par, Space const& space);

std::vector<Boid> generate_boid(int n, double v_min, double v_max,
                                Space const& space,
                                std::default_random_engine& engine);

std::vector<int> neighbours_control(int boid_to_check, double d,
                                    std::vector<Boid> const& boids,
                                    Space const& space);
Velocity separation(double s, double d_s, int boid_to_check,
                    std::vector<int> const& neighbours,
                    std::vector<Boid> const& boids, Space const& space);
Velocity alignment(double a, int boid_to_check,
                   std::vector<int> const& neighbours,
                   std::vector<Boid> const& boids);

Velocity cohesion(double c, int boid_to_check,
                  std::vector<int> const& neighbours,
                  std::vector<Boid> const& boids, Space const& space);

Velocity limit_speed(double v_min, double v_max, Velocity v_tot);

class Flock {
  // valori da mettere in input, inizializzati nel private e definiti col
  // costruttore nel public (così da poter mettere le invarianti e poterli
  // riempire nel main)
 private:
  Parameters par_;
  Space space_;
  std::vector<Boid> boids_;
  std::default_random_engine engine_;

 public:
  Flock(int n, Parameters const& par, Space const& space);

  std::vector<Boid> const& boids();
  Parameters const& parameters();
  Space const& space();


  void movement();
};

}  // namespace pf

#endif