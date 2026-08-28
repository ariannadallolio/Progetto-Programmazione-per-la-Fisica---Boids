#include "input_parameters.hpp"

#include <fstream>
#include <stdexcept>

namespace pf {
void manual_input_boids(Parameters& par, Space& space) {
  read_parameters("Number of boids: ", par.n_boids);
  read_parameters("Separation factor s: ", par.s);
  read_parameters("Alignment factor a: ", par.a);
  read_parameters("Cohesion factor c: ", par.c);
  read_parameters("Perception radius d: ", par.d);
  read_parameters("Separation radius d_s: ", par.d_s);
  read_parameters("Minimum velocity v_min: ", par.v_min);
  read_parameters("Maximum velocity v_max: ", par.v_max);
  read_parameters("Time step dt: ", par.dt);
  read_parameters("Coordinate x min of the screen: ", space.x_min);
  read_parameters("Coordinate x max of the screen: ", space.x_max);
  read_parameters("Coordinate y min of the screen: ", space.y_min);
  read_parameters("Coordinate y max of the screen: ", space.y_max);
}
void manual_input_predator(Predator_parameters& par_p) {
  read_parameters("Number of predators: ", par_p.n_predators);
  read_parameters("Predator separation factor s_p: ", par_p.s_p);
  read_parameters("Predator cohesion factor c_p: ", par_p.c_p);
  read_parameters("Predator distance d_chase: ", par_p.d_chase);
  read_parameters("Boids distance d_escape: ", par_p.d_escape);
  read_parameters("Predator minimum velocity v_min_p: ", par_p.v_min_p);
  read_parameters("Predator maximum velocity v_max_p: ", par_p.v_max_p);
}

void file_input(std::string const& file_name, Parameters& par, Space& space,
                Predator_parameters& par_p, bool has_predator) {
  std::ifstream file(file_name);
  if (!file.is_open()) {
    throw std::runtime_error{"Impossible to open the file: " + file_name};
  }
  std::string label;
  if (!(file >> label >> par.n_boids >> label >> par.s >> label >> par.a >>
        label >> par.c >> label >> par.d >> label >> par.d_s >> label >>
        par.v_min >> label >> par.v_max >> label >> par.dt >> label >>
        space.x_min >> label >> space.x_max >> label >> space.y_min >> label >>
        space.y_max)) {
    throw std::runtime_error{"Error: Missing parameters or wrong file format"};
  }

  if (has_predator) {
    if (!(file >> label >> par_p.n_predators >> label >> par_p.s_p >> label >>
          par_p.c_p >> label >> par_p.d_chase >> label >> par_p.d_escape >>
          label >> par_p.v_min_p >> label >> par_p.v_max_p)) {
      throw std::runtime_error{
          "Error: Missing parameters or wrong file format"};
    }
  }
}
}  // namespace pf