#include "simulation.hpp"

#include "statistics.hpp"
namespace pf {
void update_simulation(Flock& simulation_flock, int frame_count,
                       std::ofstream& file) {
  simulation_flock.movement();
  Statistics history =
      statistics(simulation_flock.boids(), simulation_flock.space());

  ++frame_count;
  save_for_root(history, file, frame_count);
  int const print_every = 60;  // frames
  if (frame_count % (print_every) == 0) {
    double seconds = frame_count * simulation_flock.parameters().dt;
    print(history, seconds);  // printing datas just 1 time per second, while
                              // we save datas on a "statistics.txt" for every
                              // frame to create reliable graphics
  }
}
}  // namespace pf