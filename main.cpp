#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "flock.cpp"
#include "statistics.cpp"

int main() {
  try {
    double const x_min = 0.0;
    double const x_max = 800;
    double const y_min = 0.0; 
    double const y_max = 600;

    int n{};
    int ngen{};
    std::cout << "How many boids?" << '\n';
    if (!(std::cin >> n)) {
      throw std::runtime_error{
          "Error! The number of boids has to be an integer"};
    }
    if (n <= 0) {
      throw std::runtime_error{
          "Error! The number of boids has to be positive."};
    }
    std::cout << "How many iterations?" << '\n';

    if (!(std::cin >> ngen)) {
      throw std::runtime_error{
          "Error! The number of iterations has to be an integer."};
    }
    if (ngen <= 0) {
      throw std::runtime_error{
          "Error! The number of iterations has to be positive."};
    }

    pf::Flock prova(n, 0.05, 0.05, 0.005, 100, 20, 1.0, x_min, x_max, y_min,
                    y_max);  // oppure da dare in input con txt

    for (int i = 0; i != ngen; ++i) {
      prova.movement();
      pf::print(prova.boids(), x_min, x_max, y_min,
                y_max);  // richiamo funzione esterna al flock e le passo
                         // una funzione interna al flock
    }
  } catch (std::exception const& e) {  // Cattura runtime_error
    std::cerr << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Eccezione sconosciuta\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
