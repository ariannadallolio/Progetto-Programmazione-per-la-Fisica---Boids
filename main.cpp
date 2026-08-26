
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "boids.hpp"
#include "flock.hpp"
#include "rendering.hpp"
#include "histogram.hpp"

int main() {
  try {
    pf::Parameters par;
    pf::Space space;
    int n{};

    std::cout << "Do you wanto to insert the parameters manually? (Y/n)";

    std::string answer{};
    if (!(std::cin >> answer)) {
      throw std::runtime_error{"Error: The input is not valid"};
    }

    // Controllo che la risposta sia valida
    if (answer != "Y" && answer != "y" && answer != "N" && answer != "n") {
      throw std::runtime_error{"Error: insert only Y, y, N or n"};
    }

    if (answer == "n" || answer == "N") {
      std::string input_file;
      std::cout << "Insert file name: ";
      std::cin >> input_file;

      std::ifstream file(input_file);

      if (!file.is_open()) {
        throw std::runtime_error{"Impossibile aprire il file: " + input_file};
      }

      std::string label;
      if (!(file >> label >> n >> label >> par.s >> label >> par.a >> label >>
            par.c >> label >> par.d >> label >> par.d_s >> label >> par.v_min >>
            label >> par.v_max >> label >> par.dt >> label >> space.x_min >>
            label >> space.x_max >> label >> space.y_min >> label >>
            space.y_max)) {
        throw std::runtime_error{
            "Error: Missing parameters or wrong file format"};
      }
    }

    if (answer == "y" || answer == "Y") {
      std::cout << "Number of boids: ";
      if (!(std::cin >> n)) {
        throw std::runtime_error{
            "Error: The parameter has to be an integer"};
      }

      std::cout << "Separation factor s: ";
      if (!(std::cin >> par.s)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "Alignment factor a: ";
      if (!(std::cin >> par.a)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "Cohesion factor c: ";
      if (!(std::cin >> par.c)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "Perception radius d: ";
      if (!(std::cin >> par.d)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "Separation radius d_s: ";
      if (!(std::cin >> par.d_s)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "Minimum velocity v_min: ";
      if (!(std::cin >> par.v_min)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "Maximum velocity v_max: ";
      if (!(std::cin >> par.v_max)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "Time step dt: ";
      if (!(std::cin >> par.dt)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "x_min: ";
      if (!(std::cin >> space.x_min)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "x_max: ";
      if (!(std::cin >> space.x_max)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "y_min: ";
      if (!(std::cin >> space.y_min)) {
        throw std::runtime_error{"Error: Invalid input"};
      }

      std::cout << "y_max: ";
      if (!(std::cin >> space.y_max)) {
        throw std::runtime_error{"Error: Invalid input"};
      }
    }

    pf::Flock simulation_flock(n, par,
                               space);
    pf::simulation(simulation_flock);
    pf::graph();

  } catch (std::exception const& e) {  // Cattura runtime_error
    std::cerr << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Eccezione sconosciuta\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
