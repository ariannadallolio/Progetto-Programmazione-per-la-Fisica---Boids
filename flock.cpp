#include "flock.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <random>
#include <stdexcept>

namespace pf {

// cambio il nome della funzione da generate_n a generate_boid perchè senno
// sembra un algoritmo std aggiungerei anche dei limiti alla velocità, tipo da
// v_min a v_max, lo consiglia anche nel sito
std::vector<Boid> generate_boid(int n, double v_max, double x_min, double x_max,
                                double y_min,
                                double y_max) {  // v_max serve? controlla
  assert(n > 0);  // dici che serve questo? non facciamo già il controllo nel
                  // costruttore?
  assert(x_min < x_max);
  assert(y_min < y_max);
  std::vector<Boid> boids;

  std::random_device r;  // seed
  std::default_random_engine eng{r()};
  std::uniform_real_distribution<double> uniform_v_modulus{0.0, v_max};
  double const pi = std::acos(-1.0);  // funzione arcocoseno
  std::uniform_real_distribution<double> uniform_v_angle{0.0, 2.0 * pi};
  std::uniform_real_distribution<double> uniform_px{x_min, x_max};
  std::uniform_real_distribution<double> uniform_py{y_min, y_max};
  std::generate_n(std::back_inserter(boids), n, [&]() {
    return Boid{{uniform_v_modulus(eng) * std::cos(uniform_v_angle(eng)),
                 uniform_v_modulus(eng) * std::sin(uniform_v_angle(eng))},
                {uniform_px(eng), uniform_py(eng)}};
  });
  return boids;
}

std::vector<int> neighbours_control(int boid_to_check, double d,
                                    std::vector<Boid> const& boids,
                                    double x_min, double x_max, double y_min,
                                    double y_max) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  std::vector<int> neighbours{};  // int restituisce indici boid vicini
  double d_squared = d * d;
  int const n = static_cast<int>(boids.size());
  for (int i = 0; i != n; ++i) {
    if (i != boid_to_check &&
        toroidal_distance_squared(boids[static_cast<std::size_t>(boid_to_check)]
                                      .pos,  // distanza toroidale
                                  boids[static_cast<std::size_t>(i)].pos, x_min,
                                  x_max, y_min, y_max) < d_squared) {
      neighbours.push_back(i);
    }
  }
  return neighbours;
}

Velocity separation(double s, double d_s, int boid_to_check,
                    std::vector<int> const& neighbours,
                    std::vector<Boid> const& boids, double x_min, double x_max,
                    double y_min, double y_max) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  Velocity v1{};
  if (neighbours.empty()) {
    return v1;
  }
  Position sum{0.0, 0.0};
  double d_s_squared = d_s * d_s;
  Position const pos_check = boids[static_cast<std::size_t>(boid_to_check)].pos;
  for (int m : neighbours) {  // range for loop che itera direttamente
                              // sugli elementi
    Position const pos_m = boids[static_cast<std::size_t>(m)].pos;
    if (toroidal_distance_squared(pos_m, pos_check, x_min, x_max, y_min,
                                  y_max) < d_s_squared) {
      Position const diff =
          toroidal_difference(pos_m, pos_check, x_min, x_max, y_min, y_max);
      sum += diff;  ////////////algoritmo accumulate?
    }
  }
  Position const pos_v1 = -s * sum;
  v1 = {pos_v1.x, pos_v1.y};
  return v1;
}

Velocity alignment(double a, int boid_to_check,
                   std::vector<int> const& neighbours,
                   std::vector<Boid> const& boids) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  Velocity v2{};
  if (neighbours.empty()) {
    return v2;
  }
  Velocity sum{0.0, 0.0};
  for (int m : neighbours) {
    sum += (boids[static_cast<std::size_t>(m)].vel -
            boids[static_cast<std::size_t>(boid_to_check)].vel);
  }
  int const n = static_cast<int>(neighbours.size());
  v2 = a * ((1.0 / n) * sum);  // const?

  return v2;
}

Velocity cohesion(double c, int boid_to_check,
                  std::vector<int> const& neighbours,
                  std::vector<Boid> const& boids, double x_min, double x_max,
                  double y_min, double y_max) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  Velocity v3{};
  if (neighbours.empty()) {
    return v3;
  }
  Position sum{0.0, 0.0};
  for (int m : neighbours) {
    sum +=
        toroidal_difference(boids[static_cast<std::size_t>(m)].pos,
                            boids[static_cast<std::size_t>(boid_to_check)].pos,
                            x_min, x_max, y_min, y_max);
  }
  int const n = static_cast<int>(neighbours.size());
  Position const cm = ((1.0 / n) * sum);
  Position const v3_pos = c * cm;
  v3 = {v3_pos.x, v3_pos.y};
  return v3;
}

Velocity limit_speed(double v_min, double v_max, Velocity v_tot) {
  double const modulus = speed_modulus(v_tot);
  if (modulus == 0.0) {
    return v_tot;
  }
  if (v_max < modulus) {
    return (v_max / modulus) * v_tot;
  }
  if (modulus < v_min) {
    return (v_min / modulus) * v_tot;
  }
  return v_tot;
}

Flock::Flock(int n, double s, double a, double c, double d, double d_s,
             double v_min, double v_max, double dt, double x_min, double x_max,
             double y_min, double y_max)
    : n_{n},
      s_{s},
      a_{a},
      c_{c},
      d_{d},
      d_s_{d_s},
      v_min_{v_min},
      v_max_{v_max},
      dt_{dt},
      x_min_{x_min},
      x_max_{x_max},
      y_min_{y_min},
      y_max_{y_max}

{  // ora stabiliamo l'invariante di classe con exceptions
  if (n <= 0) {
    throw std::runtime_error{
        "Errore: il numero di boids deve essere maggiore di zero"};
  }
  if (d_ <= d_s_) {
    throw std::runtime_error{
        "Errore: il raggio di separazione (d_s) deve essere minore del "
        "raggio visivo (d)"};
  }
  if (s_ < 0 || a_ < 0 || c_ < 0 || d_ <= 0 || d_s_ <= 0 || dt_ <= 0) {
    throw std::runtime_error{"Errore: i parametri devono essere positivi"};
  }  // nel main try e catch(std::runtime_error& e){std::cerr << e.what()
     // <<
  // '\n'; return EXIT_FAILURE;}

  boids_ = generate_boid(n, v_max, x_min, x_max, y_min, y_max);
}

// getter per prendere il vettore di boid e usarlo x esempio per media e
// dev std
std::vector<Boid> const& Flock::boids() const { return boids_; }

void Flock::movement() {
  std::vector<Boid> updatedboids{boids_};
  for (int j = 0; j != n_; ++j) {
    std::vector<int> neighbours =
        neighbours_control(j, d_, boids_, x_min_, x_max_, y_min_, y_max_);
    Velocity v1 = separation(s_, d_s_, j, neighbours, boids_, x_min_, x_max_,
                             y_min_, y_max_);
    Velocity v2 = alignment(a_, j, neighbours, boids_);
    Velocity v3 =
        cohesion(c_, j, neighbours, boids_, x_min_, x_max_, y_min_, y_max_);
    auto const j_sz = static_cast<std::size_t>(j);
    Velocity v_tot = boids_[j_sz].vel + v1 + v2 + v3;
    v_tot = limit_speed(v_min_, v_max_, v_tot);
    updatedboids[j_sz].vel = v_tot;
  }
  for (int j = 0; j != n_; ++j) {
    auto const j_sz = static_cast<std::size_t>(j);
    boids_[j_sz].vel = updatedboids[j_sz].vel;
    Position newp = {boids_[j_sz].pos.x + dt_ * boids_[j_sz].vel.v_x,
                     boids_[j_sz].pos.y + dt_ * boids_[j_sz].vel.v_y};
    // controllo spazio toroidale
    newp = toroidal_space(newp, x_min_, x_max_, y_min_, y_max_);
    boids_[j_sz].pos = newp;
  }
}
}  // namespace pf
