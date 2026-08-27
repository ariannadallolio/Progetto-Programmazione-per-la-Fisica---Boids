#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "boids.hpp"
#include "flock.hpp"
#include "histogram.hpp"
#include "input_parameters.hpp"
#include "sfml_rendering.hpp"

int main() {
  try {
    pf::Parameters par;
    pf::Predator_parameters par_p;
    pf::Space space;

    std::cout <<"Do you want any predator? (Y/n) ";
    std::string answer_predator{};
    if (!(std::cin >> answer_predator)) {
      throw std::runtime_error{"Error: The input is not valid"};
    }

    if (answer_predator != "Y" && answer_predator != "y" && answer_predator != "N" && answer_predator != "n") {
      throw std::runtime_error{"Error: insert only Y, y, N or n"};
    }

    bool const has_predator=(answer_predator == "Y" || answer_predator == "y");

    std::cout <<"Do you want any predator? (Y/n) ";
    std::string answer_predator{};
    if (!(std::cin >> answer_predator)) {
      throw std::runtime_error{"Error: The input is not valid"};
    }

    if (answer_predator != "Y" && answer_predator != "y" && answer_predator != "N" && answer_predator != "n") {
      throw std::runtime_error{"Error: insert only Y, y, N or n"};
    }

    bool const has_predator=(answer_predator == "Y" || answer_predator == "y");

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
      pf::file_input(input_file, par, space, par_p);
    }

    if (answer == "y" || answer == "Y") {
      pf::manual_input(par, space, par_p);
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
