#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boids.hpp"

namespace pf {

struct Parameters {
  int n_boids;
  double s;
  double a;
  double c;
  double d;
  double d_s;
  double v_min;
  double v_max;
  double dt;
};

struct Predator_parameters {
  int n_predators;
  double s_p;
  double c_p;
  double d_chase;
  double d_escape;
  double v_min_p;
  double v_max_p;
};

void check_parameters(Parameters const& par, Space const& space);
void check_predator_parameters(Predator_parameters const& par_p,
                               Space const& space);
std::vector<Boid> generate_boids(int n, double v_min, double v_max,
                                Space const& space);

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

std::vector<int> preys_control(double d_chase, Boid const& predator,
                               std::vector<Boid> const& boids,
                               Space const& space);
Velocity chase(double c_p, Boid const& predator, std::vector<int> const& preys,
               std::vector<Boid> const& boids, Space const& space);
Velocity escape(double s_p, double d_escape, Boid const& boid,
                std::vector<Boid> const& predators, Space const& space);

Velocity limit_speed(double v_min, double v_max, Velocity v_tot);

class Flock {
 private:
  Parameters par_b_;
  Predator_parameters par_p_;
  Space space_;
  std::vector<Boid> boids_;
  std::vector<Boid> predators_;

  void move_boids();
  void move_predators();

 public:
  Flock(Parameters const& par, Space const& space);
  // with predator
  Flock(Parameters const& par, Space const& space,
        Predator_parameters const& par_p);

  std::vector<Boid> const& boids() const;
  std::vector<Boid> const& predators() const;
  bool has_predator() const;
  Parameters const& parameters() const;
  Predator_parameters const& predator_parameters() const;
  Space const& space() const;

  void movement();
};

}  // namespace pf

#endif