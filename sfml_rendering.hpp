#ifndef RENDERING_HPP
#define RENDERING_HPP
#include <SFML/Graphics.hpp>
#include "flock.hpp"

namespace pf {
void sync_graphics(Flock& simulation_flock,
                 std::vector<sf::ConvexShape>& boids_shapes,
                 std::vector<sf::ConvexShape>& predator_shapes);
void run_sfml(Flock& simulation_flock);
}  // namespace pf
#endif