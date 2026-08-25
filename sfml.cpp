#include <SFML/Graphics.hpp>
#include <cmath>
#include "statistics.hpp"
#include "flock.hpp"
#include "sfml.hpp"
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
    triangle.setPointCount(3);  // creazione 3 vertici
    // creazione triangolo, SFML USA I FLOAT
    triangle.setPoint(0, sf::Vector2f(12.f, 0.f));
    triangle.setPoint(1, sf::Vector2f(-8.f, -6.f));
    triangle.setPoint(2, sf::Vector2f(-8.f, 6.f));  //  l'angolo
    // di 0 gradi corrisponde sempre alla direzione destra, conviene
    // disegnarlo già con la punta a dx così che segua la direzione
    triangle.setFillColor(sf::Color::Green);
    triangle.setOrigin(
        {0.f, 0.f});  // per come abbiamo costruito i vertici è comodo
    triangles.push_back(triangle);
  }

  // GAME LOOP
  int frame_count = 0;
  int const print_every = 60;  // con 60 fps, equivale a "una volta al secondo"

  while (window.isOpen()) {
    sf::Event event{};

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    // aggiorna simulazione
    simulation_flock.movement();
    // aggiorna contatore frame e quando raggiunge multipli di 60 stampa
    // media e dev
    ++frame_count;
    if (frame_count % (print_every) == 0) {
      print(simulation_flock.boids(), simulation_flock.space());
    }

    // aggiorna triangoli
    std::vector<pf::Boid> const& boids = simulation_flock.boids();

    for (std::size_t i = 0; i != boids.size(); ++i) {
      pf::Boid const& boid = boids[i];
      // Posizione del boid
      triangles[i].setPosition(static_cast<float>(boid.pos.x),
                               static_cast<float>(boid.pos.y));

      // direzione del boid

      double const angle_rad = std::atan2(
          boid.vel.v_y,
          boid.vel.v_x);  // il - perchè la y cresce verso il basso su SFML,
                          // così abbiamo la direzione giusta

      double const angle_deg = angle_rad * 180.0 / std::acos(-1.0);  // pi greco

      triangles[i].setRotation(static_cast<float>(angle_deg));
    }

    // disegno

    window.clear(sf::Color(20, 20,
                           30));  // crea un colore usando il modello RGB
                                  // (questo è molto scuro)

    for (sf::ConvexShape const& triangle : triangles) {
      window.draw(triangle);
    }

    window.display();
  }
}
}  // namespace pf