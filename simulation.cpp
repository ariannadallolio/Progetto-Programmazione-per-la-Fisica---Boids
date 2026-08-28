#include "simulation.hpp"

#include "statistics.hpp"

namespace pf {

void update_simulation(Flock& simulation_flock, int& frame_count,
                       std::ofstream& file) {
  simulation_flock.movement();
  Statistics const history =
      statistics(simulation_flock.boids(), simulation_flock.space());

  ++frame_count;
  double const time = frame_count * simulation_flock.parameters().dt;
  save_for_root(history, file, time);

  int const print_every = 60;  // frames
  if (frame_count % print_every == 0) {
    print(history, time);
  }
}
}  // namespace pf