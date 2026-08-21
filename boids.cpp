#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>  //per exit failure
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

struct Velocity {
  double v_x;
  double v_y;
};

struct Position {
  double x;
  double y;
};

struct Boid {
  Velocity vel;
  Position pos;
};

Position operator-(Position const& a, Position const& b) {
  return {a.x - b.x, a.y - b.y};
}

Position operator+(Position const& a, Position const& b) {
  return {a.x + b.x, a.y + b.y};
}

bool operator==(Position const& a, Position const& b) {
  return {a.x == b.x && a.y == b.y};
}

bool operator!=(Position const& a, Position const& b) { return {!(a == b)}; }

Position operator*(double c, Position const& a) { return {c * a.x, c * a.y}; }

Velocity operator-(Velocity const& a, Velocity const& b) {
  return {a.v_x - b.v_x, a.v_y - b.v_y};
}

Velocity operator+(Velocity const& a, Velocity const& b) {
  return {a.v_x + b.v_x, a.v_y + b.v_y};
}

Velocity operator*(double c, Velocity const& a) {
  return {c * a.v_x, c * a.v_y};
}

double distance_squared(Position const& a, Position const& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return dx * dx + dy * dy; //così evitiamo std::sqrt che è più impegnativo, tanto è uguale
}

double speed_modulus(Velocity const& a) {
  return std::sqrt(a.v_x * a.v_x + a.v_y * a.v_y);
}

// velocity vector: n entires, each with two coordinates (x, y)
std::vector<Boid> generate_n(int n, double x_min, double x_max, double y_min,
                             double y_max) {  // "cin n" in the main
  std::vector<Boid> boids;

  std::random_device r;  // seed
  std::default_random_engine eng{r()};
  std::uniform_real_distribution<double> uniform_v{-3.0, 3.0}; //da riguardare se includere v_max?
  // max velocity?
  std::uniform_real_distribution<double> uniform_px{x_min, x_max};
  std::uniform_real_distribution<double> uniform_py{y_min, y_max};
  std::generate_n(std::back_inserter(boids), n, [&]() {
    return Boid{{uniform_v(eng), uniform_v(eng)},
                {uniform_px(eng), uniform_py(eng)}};
  });
  return boids;
}


std::vector<int> neighbours_control(int boid_to_check, double d,
                                    std::vector<Boid> const& boids) {
  std::vector<int> neighbours{};
  double d_squared = d*d;
  int const n = static_cast<int>(boids.size());
  for (int i = 0; i != n; ++i) {
    if (i != boid_to_check &&
        distance_squared(boids[static_cast<std::size_t>(boid_to_check)].pos,
                         boids[static_cast<std::size_t>(i)].pos) < d_squared) {
      neighbours.push_back(i);
    }
  }
  return neighbours;
}

Velocity separation(double s, double d_s, int boid_to_check,
                    std::vector<int> const& neighbours,
                    std::vector<Boid> const& boids) {
  Position sum{0.0, 0.0};
  double d_s_squared = d_s * d_s;
  Position const pos_check = boids[static_cast<std::size_t>(boid_to_check)].pos;
  for (int m : neighbours) {  // range for loop che itera direttamente
                              // sugli elementi
    Position const pos_m = boids[static_cast<std::size_t>(m)].pos;
    if (distance_squared(pos_check, pos_m) < d_s_squared) {
      sum = sum + (pos_m - pos_check);  ////////////algoritmo accumulate?
    }
  }
  Position const pos_v1 = -s * sum;  // perchè const??
  Velocity v1{pos_v1.x, pos_v1.y};
  return v1;
}

Velocity alignment(double a, int boid_to_check,
                   std::vector<int> const& neighbours,
                   std::vector<Boid> const& boids) {
  Velocity v2{};
  if (neighbours.empty()) {
    return v2;
  }
  Velocity sum{0.0, 0.0};
  for (int m : neighbours) {
    sum = sum + (boids[static_cast<std::size_t>(m)].vel -
                 boids[static_cast<std::size_t>(boid_to_check)].vel);
  }
  int const n = static_cast<int>(neighbours.size());
  v2 = a * ((1.0 / n) * sum);  // const?

  return v2;
}

Velocity cohesion(double c, int boid_to_check,
                  std::vector<int> const& neighbours,
                  std::vector<Boid> const& boids) {
  Velocity v3{};
  if (neighbours.empty()) {
    return v3;
  }
  Position sum{0.0, 0.0};
  for (int m : neighbours) {
    sum = sum + boids[static_cast<std::size_t>(m)].pos;
  }
  int const n = static_cast<int>(neighbours.size());
  Position const cm = ((1.0 / n) * sum);
  Position const v3_pos =
      c * (cm - boids[static_cast<std::size_t>(boid_to_check)].pos);
  v3 = {v3_pos.x, v3_pos.y};
  return v3;
}

class Flock {
  // valori da mettere in input, inizializzati nel private e riempiti col
  // costruttore nel public (così da poter mettere le invarianti e poterli
  // riempire nel main)
 private:
  int n_;     // numero di boids
  double s_;  //<1
  double a_;  //<1
  double c_;  //<1
  double d_;
  double d_s_;  //<d
  double v_max_;
  double
      dt_;  // istante di tempo ogni quanto si aggiornano velocità e posizione

  std::vector<Boid> boids;  // vettore da riempire con il costruttore

  //dimensioni schermo in pixel
  double const x_min = 0.0;
  double const x_max = 800;
  double const y_min = 0.0;
  double const y_max = 600;

 public:
  // costruttore, Member Initilisation List
  Flock(int n, double s, double a, double c, double d, double d_s, double v_max,
        double dt)
      : n_{n},
        s_{s},
        a_{a},
        c_{c},
        d_{d},
        d_s_{d_s},
        v_max_{v_max},
        dt_{dt}

  {  // ora stabiliamo l'invariante di classe con exceptions
    if (n <= 0) {
      throw std::runtime_error{
          "Errore: il numero di boids deve essere maggiore di zero"};
    }
    if (d_ <= d_s_) {
      throw std::runtime_error{
          "Errore: il raggio di separazione (d_s) deve essere minore del "
          "raggio visivo (d)"};
    }
    if (s_ <= 0 || a_ <= 0 || c_ <= 0 || d_ <= 0 || d_s_ <= 0) {
      throw std::runtime_error{"Errore: i parametri devono essere positivi"};
    }  // nel main try e catch(std::runtime_error& e){std::cerr << e.what() <<
    // '\n'; return EXIT_FAILURE;}

    boids = generate_n(n, x_min, x_max, y_min, y_max);
  }

  Velocity limit_speed(double v_max, Velocity v_tot, double speed_tot) {
    v_tot.v_x = (v_tot.v_x / speed_tot) *
                v_max;  // creo versore modulo 1 e direzione uguale a
                         // v_tot, poi lo moltiplico per v_max così da
                         // avere modulo v_max e direzione v_tot
    v_tot.v_y = (v_tot.v_y / speed_tot) * v_max;
    return v_tot;
  }

  Position toroidal_space(Position newp) {
    if (newp.x < x_min) {
      newp.x = x_max + newp.x;
    }
    if (newp.y < y_min) {
      newp.y = y_max + newp.y;
    }
    if (x_max < newp.x) {
      newp.x = x_min + newp.x;
    }
    if (y_max < newp.y) {
      newp.y = y_min + newp.y;
    }
    return newp;
  }

  void print() const {
    for (int j = 0; j != n_; ++j) {
      std::cout << j << "° boid: " << '\n';
      auto j_sz = static_cast<std::size_t>(j);
      std::cout << "Velocity: " << boids[j_sz].vel.v_x << ","
                << boids[j_sz].vel.v_y << '\n';
      std::cout << "Position: " << boids[j_sz].pos.x << "," << boids[j_sz].pos.y
                << "\n \n \n";
    }
  }

  void movement() {
    std::vector<Boid> updatedboids{boids};
    for (int j = 0; j != n_; ++j) {
      std::vector<int> neighbours = neighbours_control(j, d_, boids);
      if (!neighbours.empty()) {
        Velocity v1 = separation(s_, d_s_, j, neighbours, boids);
        Velocity v2 = alignment(a_, j, neighbours, boids);
        Velocity v3 = cohesion(c_, j, neighbours, boids);
        auto const j_sz = static_cast<std::size_t>(j);
        Velocity v_tot = boids[j_sz].vel + v1 + v2 + v3;
        double speed_tot = speed_modulus(v_tot);
        // condizione velocità massima
        if (v_max_ < speed_tot) {
         v_tot = limit_speed(v_max_, v_tot, speed_tot);
        }
        updatedboids[j_sz].vel = v_tot;
      }
    }
    for (int j = 0; j != n_; ++j) {
      auto const j_sz = static_cast<std::size_t>(j);
      boids[j_sz].vel = updatedboids[j_sz].vel;
      Position newp = {boids[j_sz].pos.x + dt_ * boids[j_sz].vel.v_x,
                       boids[j_sz].pos.y + dt_ * boids[j_sz].vel.v_y};
      //controllo spazio toroidale
      newp = toroidal_space(newp);
      boids[j_sz].pos = newp;
    }
  }
};

int main() {
  try {
    int n{};
    int ngen{};
    std::cout << "Quanti boids?" << '\n';
    std::cin >> n;
    std::cout << "Quante iterazioni?" << '\n';
    std::cin >> ngen;
    Flock prova(n, 0.5, 0.5, 0.5, 100, 30, 10,
                0.1);  // oppure da dare in input con txt
    for (int i = 0; i != ngen; ++i) {
      prova.movement();
      prova.print();
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
