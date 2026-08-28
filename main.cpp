#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "flock.hpp"
#include "graphs.hpp"
#include "input_parameters.hpp"
#include "sfml_rendering.hpp"

int main() {
  try {
    pf::Parameters par{};
    pf::Predator_parameters par_p{};
    pf::Space space{};

    bool const has_predator = pf::ask_yes_no("Do you want any predator?");
    bool const manual =
        pf::ask_yes_no("Do you want to insert the parameters manually?");

    if (manual) {
      pf::manual_input_boids(par, space);
      if (has_predator) {
        pf::manual_input_predator(par_p);
      }
    } else {
      std::string input_file;
      std::cout << "Insert file name: ";
      std::cin >> input_file;
      pf::file_input(input_file, par, space, par_p, has_predator);
    }

    pf::Flock simulation_flock =
        has_predator ? pf::Flock(par, space, par_p) : pf::Flock(par, space);
    pf::run_sfml(simulation_flock);
    pf::draw_graphs();

  } catch (std::exception const& e) {
    std::cerr << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Unknown exception \n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}