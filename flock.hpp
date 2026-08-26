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

struct Predator_parameters {
   double s_p = 0.10;        // fattore di separazione dal predatore
  double c_p = 0.005;       // fattore di coesione del predatore
  double d_chase = 300.0;   // raggio di azione del predatore
  double d_escape = 120.0;  // raggio di percezione dei boid del predatore
  double v_min_p = 7.0;     // velocita' minima predatore
  double v_max_p = 14.0;    // velocita' massima predatore
};

void check_parameters(Parameters const& par, Space const& space);
void check_predator_parameters(Predator_parameters const& par_p,
                               Space const& space);
std::vector<Boid> generate_boid(int n, double v_min, double v_max,
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
                Boid const& predator, Space const& space);

Velocity limit_speed(double v_min, double v_max, Velocity v_tot);

class Flock {
 private:
  Parameters par_;
  Predator_parameters par_p_;
  Space space_;
  std::vector<Boid> boids_;
  Boid predator_;

 public:
  Flock(int n, Parameters const& par, Space const& space,
        Predator_parameters const& par_p = Predator_parameters{});

std::vector<Boid> const& boids() const;
  Boid const& predator() const;
  Parameters const& parameters() const;
  Predator_parameters const& predator_parameters() const;
  Space const& space() const;

  void movement();
};

}  // namespace pf

#endif