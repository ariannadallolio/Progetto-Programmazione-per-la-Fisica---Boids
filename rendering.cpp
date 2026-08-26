#include "rendering.hpp"

#include <SFML/Graphics.hpp>
#include <cmath>
#include <fstream>

#include "flock.hpp"
#include "statistics.hpp"

namespace pf {
void simulation(Flock& simulation_flock) {
  sf::RenderWindow window(
      sf::VideoMode(static_cast<unsigned int>(simulation_flock.space().x_max),
                    static_cast<unsigned int>(simulation_flock.space().y_max)),
      "Boids");
  if (!window.isOpen()) {
    throw std::runtime_error{"Errore: impossibile aprire la finestra SFML"};
  }
  window.setFramerateLimit(60);
  std::vector<sf::ConvexShape> triangles;
  triangles.reserve(simulation_flock.boids()
                        .size());  // serve a preparare lo spazio nel vettore
  int const n = static_cast<int>(simulation_flock.boids().size());
  for (int i = 0; i != n; ++i) {
    sf::ConvexShape triangle;
    triangle.setPointCount(3);
    triangle.setPoint(0, sf::Vector2f(12.f, 0.f));
    triangle.setPoint(1, sf::Vector2f(-8.f, -6.f));
    triangle.setPoint(2, sf::Vector2f(-8.f, 6.f));
    triangle.setFillColor(sf::Color::Green);
    triangle.setOrigin({0.f, 0.f});
    triangles.push_back(triangle);
  }

  // GAME LOOP
  std::ofstream file("statistics.txt");
  if (!file) {
    throw std::runtime_error{"Impossible to create file!"};
  }

  int frame_count = 0;
  int const print_every = 60;  // frame

  while (window.isOpen()) {
    sf::Event event{};

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    simulation_flock.movement();
    Statistics history =
        statistics(simulation_flock.boids(), simulation_flock.space());

    ++frame_count;
    save_for_root(history, file, frame_count);

    if (frame_count % (print_every) == 0) {
      print(history);  // printing datas just 1 time per second, while we save
                       // datas on a txt document for every frame to create
                       // reliable histograms
    }

    std::vector<pf::Boid> const& boids = simulation_flock.boids();

    for (std::size_t i = 0; i != boids.size(); ++i) {
      pf::Boid const& boid = boids[i];
      // Posizione del boid
      triangles[i].setPosition(static_cast<float>(boid.pos.x),
                               static_cast<float>(boid.pos.y));

      double const angle_rad = std::atan2(boid.vel.v_y, boid.vel.v_x);

      double const angle_deg = angle_rad * 180.0 / std::acos(-1.0);

      triangles[i].setRotation(static_cast<float>(angle_deg));
    }

    window.clear(sf::Color(20, 20, 30));
    for (sf::ConvexShape const& triangle : triangles) {
      window.draw(triangle);
    }

    window.display();
  }
}
}  // namespace pf