
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include "boids.hpp"
#include "flock.hpp"
#include "sfml.hpp"

int main() {
  try {
    std::string input_file;
    std::cout << "Insert file name: ";
    std::cin >> input_file;

    std::ifstream file(input_file);

    if (!file.is_open()) {
      throw std::runtime_error{"Impossibile aprire il file: " + input_file};
    }
    pf::Parameters par;
    pf::Space space;
    int n{};
    std::string label;
    if (!(file >> label >> n >> label >> par.s >> label >> par.a >> label >>
          par.c >> label >> par.d >> label >> par.d_s >> label >> par.v_min >>
          label >> par.v_max >> label >> par.dt >> label >> space.x_min >>
          label >> space.x_max >> label >> space.y_min >> label >>
          space.y_max)) {
      throw std::runtime_error{
          "Errore: Dati mancanti o formato errato nel file"};
    }

    pf::Flock simulation_flock(n, par, space);  // oppure da dare in input con txt

    pf::simulation(simulation_flock);

  } catch (std::exception const& e) {  // Cattura runtime_error
    std::cerr << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Eccezione sconosciuta\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
