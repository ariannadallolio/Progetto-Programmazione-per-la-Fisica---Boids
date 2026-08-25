#include "flock.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>

namespace pf {

void check_parameters(Parameters const& par, Space const& space) {
  if (!is_valid(space)) {
    throw std::invalid_argument{
        "Errore: deve essere x_min < x_max e y_min < y_max"};
  }
  if (par.s <= 0.0 || par.a <= 0.0 || par.c <= 0.0) {
    throw std::invalid_argument{
        "Errore: i fattori s, a, c devono essere positivi"};
  }
  if (par.a >= 1.0) {
    throw std::invalid_argument{
        "Errore: il fattore di allineamento a deve essere minore di 1"};
  }
  if (par.d <= 0.0 || par.d_s <= 0.0) {
    throw std::invalid_argument{
        "Errore: i raggi d e d_s devono essere "
        "positivi"};
  }
  if (par.d_s >= par.d) {
    throw std::invalid_argument{
        "Errore: il raggio di separazione d_s deve essere minore del raggio "
        "di percezione d"};
  }
  if (par.v_max <= 0.0) {
    throw std::invalid_argument{
        "Errore: la velocita' massima deve essere positiva"};
  }
  if (par.v_min < 0.0) {
    throw std::invalid_argument{
        "Errore: la velocita' minima non puo' essere negativa"};
  }
  if (par.v_min >= par.v_max) {
    throw std::invalid_argument{
        "Errore: la velocita' minima deve essere minore della massima"};
  }
  if (par.dt <= 0.0) {
    throw std::invalid_argument{
        "Errore: il passo temporale dt deve essere positivo"};
  }
}

// cambio il nome della funzione da generate_n a generate_boid perchè senno
// sembra un algoritmo std aggiungerei anche dei limiti alla velocità, tipo da
// v_min a v_max, lo consiglia anche nel sito
std::vector<Boid> generate_boid(int n, double v_min, double v_max,
                                Space const& space) {  // v_max serve? controlla
  assert(n > 0);  // dici che serve questo? non facciamo già il controllo nel
                  // costruttore?
  std::vector<Boid> boids;

  std::random_device r;  // seed
  std::default_random_engine eng{r()};
  std::uniform_real_distribution<double> uniform_v_modulus{v_min, v_max};
  double const pi = std::acos(-1.0);  // funzione arcocoseno
  std::uniform_real_distribution<double> uniform_v_angle{0.0, 2.0 * pi};
  std::uniform_real_distribution<double> uniform_px{space.x_min, space.x_max};
  std::uniform_real_distribution<double> uniform_py{space.y_min, space.y_max};
  std::generate_n(std::back_inserter(boids), n, [&]() {
    double const modulus = uniform_v_modulus(eng);
    double const angle = uniform_v_angle(eng);

    return Boid{{modulus * std::cos(angle), modulus * std::sin(angle)},
                {uniform_px(eng), uniform_py(eng)}};
  });
  return boids;
}

std::vector<int> neighbours_control(int boid_to_check, double d,
                                    std::vector<Boid> const& boids,
                                    Space const& space) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  std::vector<int> neighbours{};  // int restituisce indici boid vicini
  double d_squared = d * d;
  int const n = static_cast<int>(boids.size());
  for (int i = 0; i != n; ++i) {
    if (i != boid_to_check &&
        toroidal_distance_squared(boids[static_cast<std::size_t>(boid_to_check)]
                                      .pos,  // distanza toroidale
                                  boids[static_cast<std::size_t>(i)].pos,
                                  space) < d_squared) {
      neighbours.push_back(i);
    }
  }
  return neighbours;
}

Velocity separation(double s, double d_s, int boid_to_check,
                    std::vector<int> const& neighbours,
                    std::vector<Boid> const& boids, Space const& space) {
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
    if (toroidal_distance_squared(pos_m, pos_check, space) < d_s_squared) {
      Position const diff = toroidal_difference(pos_m, pos_check, space);
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
                  std::vector<Boid> const& boids, Space const& space) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  Velocity v3{};
  if (neighbours.empty()) {
    return v3;
  }
  Position sum{0.0, 0.0};
  for (int m : neighbours) {
    sum += toroidal_difference(
        boids[static_cast<std::size_t>(m)].pos,
        boids[static_cast<std::size_t>(boid_to_check)].pos, space);
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

Flock::Flock(int n, Parameters const& par, Space const& space)
    : par_{par}, space_{space} {
  if (n <= 0) {
    throw std::runtime_error{
        "Errore: il numero di boids deve essere maggiore di zero"};
  }

  check_parameters(par_, space_);

  boids_ = generate_boid(n, par_.v_min, par_.v_max, space_);
}

// getter per prendere il vettore di boid e usarlo x esempio per media e
// dev std
std::vector<Boid> const& Flock::boids() const { return boids_; }
Parameters const& Flock::parameters() const { return par_; }
Space const& Flock::space() const { return space_; }

void movement();

void Flock::movement() {
  std::vector<Boid> updatedboids{boids_};

  int const n = static_cast<int>(boids_.size());

  for (int j = 0; j != n; ++j) {
    std::vector<int> neighbours = neighbours_control(j, par_.d, boids_, space_);

    Velocity v1 = separation(par_.s, par_.d_s, j, neighbours, boids_, space_);

    Velocity v2 = alignment(par_.a, j, neighbours, boids_);

    Velocity v3 = cohesion(par_.c, j, neighbours, boids_, space_);

    auto const j_sz = static_cast<std::size_t>(j);

    Velocity v_tot = boids_[j_sz].vel + v1 + v2 + v3;

    v_tot = limit_speed(par_.v_min, par_.v_max, v_tot);

    updatedboids[j_sz].vel = v_tot;
  }

  for (int j = 0; j != n; ++j) {
    auto const j_sz = static_cast<std::size_t>(j);

    boids_[j_sz].vel = updatedboids[j_sz].vel;

    Position newp = {boids_[j_sz].pos.x + par_.dt * boids_[j_sz].vel.v_x,

                     boids_[j_sz].pos.y + par_.dt * boids_[j_sz].vel.v_y};

    newp = toroidal_space(newp, space_);

    boids_[j_sz].pos = newp;
  }
}

}  // namespace pf
