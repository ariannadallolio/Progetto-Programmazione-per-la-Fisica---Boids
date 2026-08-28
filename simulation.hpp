#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <fstream>

#include "flock.hpp"
namespace pf {
void update_simulation(Flock& simulation_flock, int& frame_count,
                       std::ofstream& file);
}
#endif