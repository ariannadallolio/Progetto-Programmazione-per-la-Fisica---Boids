#ifndef INPUT_PARAMETERS_HPP
#define INPUT_PARAMETERS_HPP

#include <iostream>
#include <string>

#include "flock.hpp"

namespace pf {
template <typename T>
void read_parameters(std::string const& prompt, T& par) {
  std::cout << prompt;
  if (!(std::cin >> par)) {
    throw std::runtime_error{"Error: Invalid input"};
  }
}

void manual_input_boids(Parameters& par, Space& space);
void manual_input_predator(Predator_parameters& par_p);

void file_input(std::string& file_name, Parameters& par, Space& space,
                Predator_parameters& par_p, bool has_predator);
}  // namespace pf

#endif