#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "boids.hpp"
#include "flock.hpp"
#include "histogram.hpp"
#include "sfml_rendering.hpp"

int main() {
  try {
    pf::Parameters par;
    pf::Predator_parameters par_p;
    pf::Space space;

    std::cout << "Do you wanto to insert the parameters manually? (Y/n)";

    std::string answer{};
    if (!(std::cin >> answer)) {
      throw std::runtime_error{"Error: The input is not valid"};
    }

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
      if (!(file >> label >> par.n_boids >> label >> par.s >> label >> par.a >>
            label >> par.c >> label >> par.d >> label >> par.d_s >> label >>
            par.v_min >> label >> par.v_max >> label >> par.dt >> label >>
            space.x_min >> label >> space.x_max >> label >> space.y_min >>
            label >> space.y_max >> label >> par_p.n_predators >> label >>
            par_p.s_p >> label >> par_p.c_p >> label >> par_p.d_chase >>
            label >> par_p.d_escape >> label >> par_p.v_min_p >> label >>
            par_p.v_max_p)) {
        throw std::runtime_error{
            "Error: Missing parameters or wrong file format"};
      }
    }

    if (answer == "y" || answer == "Y") {
      std::cout << "Number of boids: ";
      if (!(std::cin >> par.n_boids)) {
        throw std::runtime_error{"Error: The parameter has to be an integer"};
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

      if (!(std::cin >> par_p.n_predators)) {
        throw std::runtime_error{"Error: The parameter has to be an integer"};
      }
      std::cout << "Predator separation factor s_p: ";
      if (!(std::cin >> par_p.s_p)) {
        throw std::runtime_error{"Error: Invalid input"};
      }
      std::cout << "Predator separation factor c_p: ";
      if (!(std::cin >> par_p.c_p)) {
        throw std::runtime_error{"Error: Invalid input"};
      }
      std::cout << "Predator separation factor d_chase: ";
      if (!(std::cin >> par_p.d_chase)) {
        throw std::runtime_error{"Error: Invalid input"};
      }
      std::cout << "Predator separation factor d_escape: ";
      if (!(std::cin >> par_p.d_escape)) {
        throw std::runtime_error{"Error: Invalid input"};
      }
      std::cout << "Predator separation factor v_min_p: ";
      if (!(std::cin >> par_p.v_min_p)) {
        throw std::runtime_error{"Error: Invalid input"};
      }
      std::cout << "Predator separation factor v_max_p: ";
      if (!(std::cin >> par_p.v_max_p)) {
        throw std::runtime_error{"Error: Invalid input"};
      }
    }

    pf::Flock simulation_flock(par, space, par_p);
    pf::simulation(simulation_flock);
    pf::graph();

  } catch (std::exception const& e) {
    std::cerr << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Eccezione sconosciuta\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
