#include "flock.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <random>
#include <stdexcept>

namespace pf {

void check_parameters(Parameters const& par, Space const& space) {
  if (!space_is_valid(space)) {
    throw std::invalid_argument{
        "Error: It has to be x_min < x_max and y_min < y_max"};
  }
  if (par.n_boids <= 0) {
    throw std::invalid_argument{"Error: the number of boids must be positive"};
  }
  if (par.s <= 0.0 || par.a <= 0.0 || par.c <= 0.0) {
    throw std::invalid_argument{
        "Error: the parameters s, a, c must be positive"};
  }
  if (par.d <= 0.0 || par.d_s <= 0.0) {
    throw std::invalid_argument{
        "Error: the parameters d e d_s must be positive"};
  }
  if (par.d_s >= par.d) {
    throw std::invalid_argument{
        "Error: The separation radius d_s must be less than the radius of "
        "perception d"};
  }
  if (par.v_max <= 0.0) {
    throw std::invalid_argument{"Error: The maximum speed must be positive"};
  }
  if (par.v_min < 0.0) {
    throw std::invalid_argument{"Error: The minimum speed cannot be negative"};
  }
  if (par.v_min >= par.v_max) {
    throw std::invalid_argument{
        "Error: the minimum speed must be less than the maximum speed"};
  }
  if (par.dt <= 0.0) {
    throw std::invalid_argument{"Error: The time step dt must be positive"};
  }
}

void check_predator_parameters(Predator_parameters const& par_p,
                               Space const& space) {
  if (par_p.n_predators <= 0) {
    throw std::invalid_argument{
        "Error: the number of predators must be positive"};
  }
  if (par_p.s_p <= 0.0 || par_p.c_p <= 0.0) {
    throw std::invalid_argument{
        "Error: the parameters s_p and c_p must be positive"};
  }
  if (par_p.d_chase <= 0.0 || par_p.d_escape <= 0.0) {
    throw std::invalid_argument{
        "Error: the parameters d_chase e d_escape must be positive"};
  }
  if (par_p.v_min_p <= 0.0) {
    throw std::invalid_argument{
        "Error: The minimum speed of the predatore must be positive"};
  }
  if (par_p.v_max_p <= 0.0) {
    throw std::invalid_argument{
        "Error: The maximum speed of the predatore must be positive"};
  }
  if (par_p.v_min_p >= par_p.v_max_p) {
    throw std::invalid_argument{
        "Error: the minimum speed must be less than the maximum speed"};
  }

  double const Lx = space.x_max - space.x_min;
  double const Ly = space.y_max - space.y_min;

  if (par_p.d_chase > std::min(Lx, Ly) / 2.0) {
    throw std::invalid_argument{
        "Error: The hunting radius d_chase cannot exceed half of the domain"};
  }
}

std::vector<Boid> generate_boid(int n, double v_min, double v_max,
                                Space const& space) {
  assert(n > 0);
  std::vector<Boid> boids;
  boids.reserve(static_cast<std::size_t>(n));

  std::random_device r;  // seed
  std::default_random_engine eng{r()};
  std::uniform_real_distribution<double> uniform_v_modulus{v_min, v_max};
  double const pi = std::acos(-1.0);
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
  int const n = static_cast<int>(boids.size());
  assert(boid_to_check < n);

  std::vector<int> neighbours{};  // an int in "neighbours" vector represents
                                  // the equivalent position in "boids" vector
                                  // of a boid_to_check's neighbour
  if (n > 1) {
    neighbours.reserve(n - 1);
  }

  double d_squared = d * d;

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
  for (int m : neighbours) {
    Position const pos_m = boids[static_cast<std::size_t>(m)].pos;
    if (toroidal_distance_squared(pos_m, pos_check, space) < d_s_squared) {
      Position const diff = toroidal_difference(pos_m, pos_check, space);
      sum += diff;
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

std::vector<int> preys_control(double d_chase, Boid const& predator,
                               std::vector<Boid> const& boids,
                               Space const& space) {
  std::vector<int> preys{};
  int const n = static_cast<int>(boids.size());
  preys.reserve(n);
  
  double const d_chase_double = d_chase * d_chase;
  for (int i = 0; i != n; ++i) {
    if (toroidal_distance_squared(boids[static_cast<std::size_t>(i)].pos,
                                  predator.pos, space) < d_chase_double) {
      preys.push_back(i);
    }
  }
  return preys;
}

Velocity chase(double c_p, Boid const& predator, std::vector<int> const& preys,
               std::vector<Boid> const& boids, Space const& space) {
  Velocity v_c{};
  if (preys.empty()) {
    return v_c;
  }

  Position sum{0.0, 0.0};

  for (int m : preys) {
    sum += toroidal_difference(boids[static_cast<std::size_t>(m)].pos,
                               predator.pos, space);
  }
  double const n = static_cast<int>(preys.size());
  Position const center_direction = (sum * (1.0 / n));
  v_c = {c_p * center_direction.x, c_p * center_direction.y};
  return v_c;
}

Velocity escape(double s_p, double d_escape, Boid const& boid,
                std::vector<Boid> const& predators, Space const& space) {
  Velocity v4{};
  double const d_escape_squared = d_escape * d_escape;
  for (Boid const& predator : predators) {
    if (toroidal_distance_squared(boid.pos, predator.pos, space) <
        d_escape_squared) {
      Position const diff = toroidal_difference(predator.pos, boid.pos, space);
      Position const v_pos = -s_p * diff;
      v4 += Velocity{v_pos.x,
                     v_pos.y};  // sommo le velocità legate a più predatori
    }
  }
  return v4;
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

<<<<<<< HEAD
Flock::Flock(Parameters const& par, Space const& space)
    : par_b_{par}, par_p_{}, space_{space}{
=======
Flock::Flock(Parameters const& par, Space const& space,
             Predator_parameters const& par_p)
    : par_b_{par}, par_p_{par_p}, space_{space} {
>>>>>>> 4c927f4ef145cb45494ecc6e550548793796cbcb
  check_parameters(par_b_, space_);

  boids_ = generate_boid(par_b_.n_boids, par_b_.v_min, par_b_.v_max, space_);
}

// costruttore di questo Flock crea uno stormo con predatore utilizzando il
// primo costruttore
Flock::Flock(Parameters const& par, Space const& space,
             Predator_parameters const& par_p)
    : Flock(par, space) {  // costruttore delegante
  check_predator_parameters(par_p, space_);

  par_p_ = par_p;
  predators_ =
      generate_boid(par_p_.n_predators, par_p_.v_min_p, par_p_.v_max_p, space_);
}

// getter per prendere il vettore di boid e usarlo x esempio per media e
// dev std
std::vector<Boid> const& Flock::boids() const { return boids_; }
std::vector<Boid> const& Flock::predators() const { return predators_; }
bool Flock::has_predator() const { return !predators_.empty(); }
Parameters const& Flock::parameters() const { return par_b_; }
Predator_parameters const& Flock::predator_parameters() const { return par_p_; }
Space const& Flock::space() const { return space_; }

void Flock::movement() {
  std::vector<Velocity> new_velocities;
  new_velocities.reserve(boids_.size());

  int const n = static_cast<int>(boids_.size());

  for (int i = 0; i != n; ++i) {
    std::vector<int> neighbours =
        neighbours_control(i, par_b_.d, boids_, space_);

    Velocity const v1 =
        separation(par_b_.s, par_b_.d_s, i, neighbours, boids_, space_);
    Velocity const v2 = alignment(par_b_.a, i, neighbours, boids_);
    Velocity const v3 = cohesion(par_b_.c, i, neighbours, boids_, space_);
    auto const i_sz = static_cast<std::size_t>(i);
    // se predators_ e' vuoto il ciclo non parte e il contributo e' nullo
    Velocity const v4 =
        escape(par_p_.s_p, par_p_.d_escape, boids_[i_sz], predators_, space_);

    new_velocities.push_back(limit_speed(par_b_.v_min, par_b_.v_max,
                                         boids_[i_sz].vel + v1 + v2 + v3 + v4));
  }

  for (int j = 0; j != n; ++j) {
    auto const j_sz = static_cast<std::size_t>(j);
    boids_[j_sz].vel = new_velocities[j_sz];

    Position newp = {boids_[j_sz].pos.x + par_b_.dt * boids_[j_sz].vel.v_x,

                     boids_[j_sz].pos.y + par_b_.dt * boids_[j_sz].vel.v_y};

    boids_[j_sz].pos = toroidal_space(newp, space_);
  }

  // predator
  for (Boid& predator : predators_) {
    std::vector<int> const preys =
        preys_control(par_p_.d_chase, predator, boids_, space_);
    Velocity const v_chase =
        chase(par_p_.c_p, predator, preys, boids_, space_);
    predator.vel =
        limit_speed(par_p_.v_min_p, par_p_.v_max_p, predator.vel + v_chase);

    // calcolo nuova posizione predatore
    Position const newp_p = {predator.pos.x + par_b_.dt * predator.vel.v_x,
                             predator.pos.y + par_b_.dt * predator.vel.v_y};
    predator.pos = toroidal_space(newp_p, space_);
  }
}

}  // namespace pf
