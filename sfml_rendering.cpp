#include "sfml_rendering.hpp"

#include <SFML/Graphics.hpp>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "simulation.hpp"

namespace pf {

void sync_graphics(Flock& simulation_flock,
                   std::vector<sf::ConvexShape>& boids_shapes,
                   std::vector<sf::ConvexShape>& predator_shapes) {
  int const n_boids = simulation_flock.parameters().n_boids;

  for (int i = 0; i != n_boids; ++i) {
    auto const i_sz = static_cast<std::size_t>(i);
    Boid const& boid = simulation_flock.boids()[i_sz];
    boids_shapes[i_sz].setPosition(static_cast<float>(boid.pos.x),
                                   static_cast<float>(boid.pos.y));

    double const angle_rad = std::atan2(boid.vel.v_y, boid.vel.v_x);

    double const angle_deg = angle_rad * 180.0 / std::acos(-1.0);

    boids_shapes[i_sz].setRotation(static_cast<float>(angle_deg));
  }

  int const n_predators = simulation_flock.predator_parameters().n_predators;
  for (int i = 0; i != n_predators; ++i) {
    auto const i_sz = static_cast<std::size_t>(i);
    Boid const& pred = simulation_flock.predators()[i_sz];

    predator_shapes[i_sz].setPosition(static_cast<float>(pred.pos.x),
                                      static_cast<float>(pred.pos.y));

    double const pred_angle_deg =
        std::atan2(pred.vel.v_y, pred.vel.v_x) * 180.0 / std::acos(-1.0);

    predator_shapes[i_sz].setRotation(static_cast<float>(pred_angle_deg));
  }
}

void run_sfml(Flock& simulation_flock) {
  Space const& space = simulation_flock.space();
  auto const Lx = static_cast<float>(space.x_max - space.x_min);
  auto const Ly = static_cast<float>(space.y_max - space.y_min);

  sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(Lx),
                                        static_cast<unsigned int>(Ly)),
                          "Boids");

  window.setView(
      sf::View(sf::FloatRect(static_cast<float>(space.x_min),
                             static_cast<float>(space.y_min), Lx, Ly)));

  window.setFramerateLimit(60);
  std::vector<sf::ConvexShape> boids_shapes;
  boids_shapes.reserve(simulation_flock.boids().size());
  int const n = static_cast<int>(simulation_flock.boids().size());
  for (int i = 0; i != n; ++i) {
    sf::ConvexShape triangle;
    triangle.setPointCount(3);
    triangle.setPoint(0, sf::Vector2f(12.f, 0.f));
    triangle.setPoint(1, sf::Vector2f(-8.f, -6.f));
    triangle.setPoint(2, sf::Vector2f(-8.f, 6.f));
    triangle.setFillColor(sf::Color::Green);
    triangle.setOrigin({0.f, 0.f});
    boids_shapes.push_back(triangle);
  }

  std::vector<sf::ConvexShape> predator_shapes;
  predator_shapes.reserve(simulation_flock.predators().size());
  int const n_p = static_cast<int>(simulation_flock.predators().size());
  for (int i = 0; i != n_p; ++i) {
    sf::ConvexShape triangle;
    triangle.setPointCount(3);
    triangle.setPoint(0, sf::Vector2f(20.f, 0.f));
    triangle.setPoint(1, sf::Vector2f(-13.f, -10.f));
    triangle.setPoint(2, sf::Vector2f(-13.f, 10.f));
    triangle.setFillColor(sf::Color::Red);
    triangle.setOrigin({0.f, 0.f});
    predator_shapes.push_back(triangle);
  }

  std::ofstream file("statistics.txt");
  if (!file) {
    throw std::runtime_error{"Impossible to create file!"};
  }

  int frame_count = 0;

  while (window.isOpen()) {
    sf::Event event{};

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    update_simulation(simulation_flock, frame_count, file);
    sync_graphics(simulation_flock, boids_shapes, predator_shapes);

    window.clear(sf::Color(20, 20, 30));
    for (sf::ConvexShape const& triangle : boids_shapes) {
      window.draw(triangle);
    }

    for (sf::ConvexShape const& triangle : predator_shapes) {
      window.draw(triangle);
    }

    window.display();
  }
}
}  // namespace pf