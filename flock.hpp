#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boids.hpp"

namespace pf {
std::vector<Boid> generate_boid(int n, double v_max, double x_min, double x_max,
                                double y_min, double y_max);

std::vector<int> neighbours_control(int boid_to_check, double d,
                                    std::vector<Boid> const& boids,
                                    double x_min, double x_max, double y_min,
                                    double y_max);
Velocity separation(double s, double d_s, int boid_to_check,
                    std::vector<int> const& neighbours,
                    std::vector<Boid> const& boids, double x_min, double x_max,
                    double y_min, double y_max);
Velocity alignment(double a, int boid_to_check,
                   std::vector<int> const& neighbours,
                   std::vector<Boid> const& boids);

Velocity cohesion(double c, int boid_to_check,
                  std::vector<int> const& neighbours,
                  std::vector<Boid> const& boids, double x_min, double x_max,
                  double y_min, double y_max);

Velocity limit_speed(double v_min, double v_max, Velocity v_tot);

class Flock {
  // valori da mettere in input, inizializzati nel private e definiti col
  // costruttore nel public (così da poter mettere le invarianti e poterli
  // riempire nel main)
 private:
  int n_;     // numero di boids
  double s_;  //<1
  double a_;  //<1
  double c_;  //<1
  double d_;
  double d_s_;    //<d
  double v_min_;
  double v_max_;  // double v_max_;
  double
      dt_;  // istante di tempo ogni quanto si aggiornano velocità e posizione
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;

  std::vector<Boid> boids_;

  public:
  Flock(int n, double s, double a, double c, double d, double d_s, double v_min, double v_max,
        double dt, double x_min, double x_max, double y_min, double y_max);
  std::vector<Boid> const& boids() const;
  void movement();      
};

}  // namespace pf

#endif