#ifndef BOIDS_HPP
#define BOIDS_HPP

namespace pf {
struct Velocity {
  double v_x;
  double v_y;
};

struct Position {
  double x;
  double y;
};

struct Boid {
  Velocity vel;
  Position pos;
};

struct Space {
  double x_min;
  double x_max;
  double y_min;
  double y_max;
};

bool is_valid(Space const& space);

Position operator-(Position const& a, Position const& b);
Position operator+(Position const& a, Position const& b);
Position& operator+=(Position& a, Position const& b);
Position operator*(double c, Position const& a);
Position operator*(Position const& a, double c);
Velocity operator-(Velocity const& a, Velocity const& b);
Velocity operator+(Velocity const& a, Velocity const& b);
Velocity& operator+=(Velocity& a, Velocity const& b);
Velocity operator*(double c, Velocity const& a);
Velocity operator*(Velocity const& a, double c);
bool operator==(Position const& a, Position const& b);
bool operator!=(Position const& a, Position const& b);
Position toroidal_space(Position newp, Space const& space);
Position toroidal_difference(Position const& a, Position const& b,
                             Space const& space);
double toroidal_distance_squared(Position const& a, Position const& b,
                                 Space const& space);
double speed_modulus(Velocity const& a);

}  // namespace pf

#endif