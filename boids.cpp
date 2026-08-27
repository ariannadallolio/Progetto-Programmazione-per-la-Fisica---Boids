#include "boids.hpp"

#include <cmath>

namespace pf {

// space operators
bool space_is_valid(Space const& space) {
  return space.x_min < space.x_max && space.y_min < space.y_max;
}

Position operator-(Position const& a, Position const& b) {
  return {a.x - b.x, a.y - b.y};
}

Position operator+(Position const& a, Position const& b) {
  return {a.x + b.x, a.y + b.y};
}

Position& operator+=(Position& a, Position const& b) {
  a.x += b.x;
  a.y += b.y;
  return a;
}

Position operator*(double c, Position const& a) { return {c * a.x, c * a.y}; }

Position operator*(Position const& a, double c) { return c * a; }

bool operator==(Position const& a, Position const& b) {
  return (a.x == b.x && a.y == b.y);
}

bool operator!=(Position const& a, Position const& b) { return (!(a == b)); }

// velocity operators
Velocity operator-(Velocity const& a, Velocity const& b) {
  return {a.v_x - b.v_x, a.v_y - b.v_y};
}

Velocity operator+(Velocity const& a, Velocity const& b) {
  return {a.v_x + b.v_x, a.v_y + b.v_y};
}

Velocity& operator+=(Velocity& a, Velocity const& b) {
  a.v_x += b.v_x;
  a.v_y += b.v_y;
  return a;
}

Velocity operator*(double c, Velocity const& a) {
  return {c * a.v_x, c * a.v_y};
}
Velocity operator*(Velocity const& a, double c) { return c * a; }

Position toroidal_space(Position newp, Space const& space) {
  double const Lx = space.x_max - space.x_min;
  double const Ly = space.y_max - space.y_min;

  double x = std::fmod(newp.x - space.x_min, Lx);
  if (x < 0.0) {
    x += Lx;
  }

  double y = std::fmod(newp.y - space.y_min, Ly);
  if (y < 0.0) {
    y += Ly;
  }

  return {space.x_min + x, space.y_min + y};
}

Position toroidal_difference(Position const& a, Position const& b,
                             Space const& space) {
  double const Lx = space.x_max - space.x_min;
  double const Ly = space.y_max - space.y_min;
  double dx = a.x - b.x;
  double dy = a.y - b.y;

  if (dx > Lx / 2.0) {
    dx -= Lx;
  }
  if (dx < -Lx / 2.0) {
    dx += Lx;
  }
  if (dy > Ly / 2.0) {
    dy -= Ly;
  }
  if (dy < -Ly / 2.0) {
    dy += Ly;
  }
  return {dx, dy};
}

double toroidal_distance_squared(Position const& a, Position const& b,
                                 Space const& space) {
  Position const diff = toroidal_difference(a, b, space);
  return diff.x * diff.x + diff.y * diff.y;
}

double speed_modulus(Velocity const& a) {
  return std::sqrt(a.v_x * a.v_x + a.v_y * a.v_y);
}
}  // namespace pf
