#include <SFML/Graphics.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "boids.hpp"
#include "flock.hpp"
#include "statistics.hpp"

int main() {
  try {
    std::string input_file;
    std::cout << "Insert file name: ";
    std::cin >> input_file;

    std::ifstream file(input_file);

    if (!file.is_open()) {
      throw std::runtime_error{"Impossibile aprire il file: " + input_file};
    }
    pf::Parameters par;
    pf::Space space;
    int n{};
    std::string label;
    if (!(file >> label >> n >> label >> par.s >> label >> par.a >> label >>
          par.c >> label >> par.d >> label >> par.d_s >> label >> par.v_min >>
          label >> par.v_max >> label >> par.dt >> label >> space.x_min >>
          label >> space.x_max >> label >> space.y_min >> label >>
          space.y_max)) {
      throw std::runtime_error{
          "Errore: Dati mancanti o formato errato nel file"};
    }

    pf::Flock prova(n, par, space);  // oppure da dare in input con txt

    sf::RenderWindow window(
        sf::VideoMode(static_cast<unsigned int>(space.x_max),
                      static_cast<unsigned int>(space.y_max)),
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
    int const print_every =
        60;  // con 60 fps, equivale a "una volta al secondo"

    while (window.isOpen()) {
      sf::Event event{};

      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
        }
      }
      // aggiorna simulazione
      prova.movement();
      // aggiorna contatore frame e quando raggiunge multipli di 60 stampa
      // media e dev
      ++frame_count;
      if (frame_count % (print_every) == 0) {
        print(prova.boids(), space);
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

      window.clear(sf::Color(20, 20,
                             30));  // crea un colore usando il modello RGB
                                    // (questo è molto scuro)

      for (sf::ConvexShape const& triangle : triangles) {
        window.draw(triangle);
      }

      window.display();
    }
  } catch (std::exception const& e) {  // Cattura runtime_error
    std::cerr << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Eccezione sconosciuta\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
