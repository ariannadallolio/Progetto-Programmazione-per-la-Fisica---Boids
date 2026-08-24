#include <SFML/Graphics.hpp>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "boids.hpp"
#include "flock.hpp"
#include "statistics.hpp"

int main() {
  try {
    double const x_min = 0.0;
    double const x_max = 800;
    double const y_min = 0.0;
    double const y_max = 600;

    int n{};
    std::cout << "How many boids?" << '\n';
    if (!(std::cin >> n)) {
      throw std::runtime_error{
          "Error! The number of boids has to be an integer"};
    }
    if (n <= 0) {
      throw std::runtime_error{
          "Error! The number of boids has to be positive."};
    }

    pf::Flock prova(n, 0.08, 0.15, 0.003, 100, 30, 3.0, 12.0, 1.0, x_min, x_max,
                    y_min,
                    y_max);  // oppure da dare in input con txt

    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(x_max),
                                          static_cast<unsigned int>(y_max)),
                            "Boids");
    if (!window.isOpen()) {
      throw std::runtime_error{"Errore: impossibile aprire la finestra SFML"};
    }
    window.setFramerateLimit(60);
    std::vector<sf::ConvexShape> triangles;
    triangles.reserve(
        prova.boids().size());  // serve a preparare lo spazio nel vettore
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
      sf::Event event;

      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
        }
      }
      // aggiorna simulazione
      prova.movement();
      //aggiorna contatore frame e quando raggiunge multipli di 60 stampa media e dev 
      ++frame_count;
      if (frame_count%(print_every) == 0){
      print(prova.boids(), x_min, x_max, y_min, y_max);
      }

      // aggiorna triangoli
      std::vector<pf::Boid> const& boids = prova.boids();

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

        double const angle_deg =
            angle_rad * 180.0 / std::acos(-1.0);  // pi greco

        triangles[i].setRotation(static_cast<float>(angle_deg));
      }

      // disegno

      window.clear(sf::Color(
          20, 20,
          30));  // crea un colore usando il modello RGB (questo è molto scuro)

      for (sf::ConvexShape const& triangle : triangles) {
        window.draw(triangle);
      }

      window.display();
    }
  }
  /*

    prova.movement();
    pf::print(prova.boids(), x_min, x_max, y_min,
              y_max);  // richiamo funzione esterna al flock e le passo
                       // una funzione interna al flock
*/

  catch (std::exception const& e) {  // Cattura runtime_error
    std::cerr << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Eccezione sconosciuta\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
