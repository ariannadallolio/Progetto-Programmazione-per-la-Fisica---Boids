#ifndef INPUT_PARAMETERS_HPP
#define INPUT_PARAMETERS_HPP

#include <iostream>

#include "flock.hpp"

namespace pf {
template <typename T>
void read_parameters(std::string const& prompt, T& par);

void manual_input(Parameters& par, Space& space, Predator_parameters& par_p);

void file_input(std::string& file_name, Parameters& par, Space& space,
                Predator_parameters& par_p);
}  // namespace pf

#endif