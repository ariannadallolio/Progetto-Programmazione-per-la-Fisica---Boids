#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include "flock.hpp"
#include <fstream>
namespace pf{
void update_simulation(Flock& simulation_flock, int& frame_count, std::ofstream& file);
}
#endif