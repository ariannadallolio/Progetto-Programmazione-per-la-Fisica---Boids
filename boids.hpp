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

// operatori posizione

Position operator-(Position const& a, Position const& b) {
  return {a.x - b.x, a.y - b.y};
}

Position operator+(Position const& a, Position const& b) {
  return {a.x + b.x, a.y + b.y};
}

Position& operator+=(Position& a, Position const& b) {
  a.x += b.x;
  a.y += b.y;
  return a;
}

Position operator*(double c, Position const& a) { return {c * a.x, c * a.y}; }
Position operator*(Position const& a, double c) { return c * a; }

// operatori velocità

Velocity operator-(Velocity const& a, Velocity const& b) {
  return {a.v_x - b.v_x, a.v_y - b.v_y};
}

Velocity operator+(Velocity const& a, Velocity const& b) {
  return {a.v_x + b.v_x, a.v_y + b.v_y};
}

Velocity& operator+=(Velocity& a, Velocity const& b) {
  a.v_x += b.v_x;
  a.v_y += b.v_y;
  return a;
}

Velocity operator*(double c, Velocity const& a) {
  return {c * a.v_x, c * a.v_y};
}
Velocity operator*(Velocity const& a, double c) { return c * a; }

bool operator==(Position const& a, Position const& b) {
  return {a.x == b.x && a.y == b.y};
}

bool operator!=(Position const& a, Position const& b) { return {!(a == b)}; }

/*
double distance_squared(Position const& a, Position const& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return dx * dx + dy * dy;  // così evitiamo std::sqrt che è più impegnativo,
                             // tanto è uguale
}
 */

// forse questa si può togliere
/*
double distance(Position const& a, Position const& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}
  */

Position toroidal_space(Position newp, double x_min, double x_max, double y_min,
                        double y_max) {
  assert(x_min < x_max);
  assert(y_min < y_max);
  double const Lx = x_max - x_min;
  double const Ly = y_max - y_min;
  if (newp.x < x_min) {
    newp.x += Lx;
  }
  if (newp.y < y_min) {
    newp.y += Ly;
  }
  if (x_max < newp.x) {
    newp.x -= Lx;
  }
  if (y_max < newp.y) {
    newp.y -= Ly;
  }
  return newp;
}

// questa funzione definisce la distanza tra due boids nello spazio toroidale,
// calcola qual è la distanza minore, se quella a destra o a sinistra e utilizza
// quella minore
Position toroidal_difference(Position const& a, Position const& b, double x_min,
                             double x_max, double y_min, double y_max) {
  assert(x_min < x_max);
  assert(y_min < y_max);
  double const Lx = x_max - x_min;
  double const Ly = y_max - y_min;
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  // il segno + o - dipende da quanto vale la differenza di posizione tra i due
  // boid
  if (dx > Lx / 2.0) {
    dx -= Lx;
  }
  if (dx < -Lx / 2.0) {
    dx += Lx;
  }
  if (dy > Ly / 2.0) {
    dy -= Ly;
  }
  if (dy < -Ly / 2.0) {
    dy += Ly;
  }
  return {dx, dy};
}

// questa funzione calcola la distanza quadrata tra due boids, tenendo consto
// della distanza minore calcolata da toroidal_distance
double toroidal_distance_squared(Position const& a, Position const& b,
                                 double x_min, double x_max, double y_min,
                                 double y_max) {
  Position const diff = toroidal_difference(a, b, x_min, x_max, y_min, y_max);
  return diff.x * diff.x + diff.y * diff.y;
}

double speed_modulus(Velocity const& a) {
  return std::sqrt(a.v_x * a.v_x + a.v_y * a.v_y);
}

// cambio il nome della funzione da generate_n a generate_boid perchè senno
// sembra un algoritmo std aggiungerei anche dei limiti alla velocità, tipo da
// v_min a v_max, lo consiglia anche nel sito
std::vector<Boid> generate_boid(int n, double v_max, double x_min, double x_max,
                                double y_min,
                                double y_max) {  // v_max serve? controlla
  assert(n > 0);  // dici che serve questo? non facciamo già il controllo nel
                  // costruttore?
  assert(x_min < x_max);
  assert(y_min < y_max);
  std::vector<Boid> boids;

  std::random_device r;  // seed
  std::default_random_engine eng{r()};
  std::uniform_real_distribution<double> uniform_v{
      -v_max, v_max};  // da riguardare se includere v_max?
  //  max velocity?
  std::uniform_real_distribution<double> uniform_px{x_min, x_max};
  std::uniform_real_distribution<double> uniform_py{y_min, y_max};
  std::generate_n(std::back_inserter(boids), n, [&]() {
    return Boid{{uniform_v(eng), uniform_v(eng)},
                {uniform_px(eng), uniform_py(eng)}};
  });
  return boids;
}

std::vector<int> neighbours_control(int boid_to_check, double d,
                                    std::vector<Boid> const& boids,
                                    double x_min, double x_max, double y_min,
                                    double y_max) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  std::vector<int> neighbours{};  // int restituisce indici boid vicini
  double d_squared = d * d;
  int const n = static_cast<int>(boids.size());
  for (int i = 0; i != n; ++i) {
    if (i != boid_to_check &&
        toroidal_distance_squared(boids[static_cast<std::size_t>(boid_to_check)]
                                      .pos,  // distanza toroidale
                                  boids[static_cast<std::size_t>(i)].pos, x_min,
                                  x_max, y_min, y_max) < d_squared) {
      neighbours.push_back(i);
    }
  }
  return neighbours;
}

Velocity separation(double s, double d_s, int boid_to_check,
                    std::vector<int> const& neighbours,
                    std::vector<Boid> const& boids, double x_min, double x_max,
                    double y_min, double y_max) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  Velocity v1{};
  if (neighbours.empty()) {
    return v1;
  }
  Position sum{0.0, 0.0};
  double d_s_squared = d_s * d_s;
  Position const pos_check = boids[static_cast<std::size_t>(boid_to_check)].pos;
  for (int m : neighbours) {  // range for loop che itera direttamente
                              // sugli elementi
    Position const pos_m = boids[static_cast<std::size_t>(m)].pos;
    if (toroidal_distance_squared(pos_check, pos_m, x_min, x_max, y_min,
                                  y_max) < d_s_squared) {
      Position const diff =
          toroidal_difference(pos_check, pos_m, x_min, x_max, y_min, y_max);
      sum += diff;  ////////////algoritmo accumulate?
    }
  }
  Position const pos_v1 = -s * sum;
  v1 = {pos_v1.x, pos_v1.y};
  return v1;
}

Velocity alignment(double a, int boid_to_check,
                   std::vector<int> const& neighbours,
                   std::vector<Boid> const& boids) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  Velocity v2{};
  if (neighbours.empty()) {
    return v2;
  }
  Velocity sum{0.0, 0.0};
  for (int m : neighbours) {
    sum += (boids[static_cast<std::size_t>(m)].vel -
            boids[static_cast<std::size_t>(boid_to_check)].vel);
  }
  int const n = static_cast<int>(neighbours.size());
  v2 = a * ((1.0 / n) * sum);  // const?

  return v2;
}

Velocity cohesion(double c, int boid_to_check,
                  std::vector<int> const& neighbours,
                  std::vector<Boid> const& boids, double x_min, double x_max,
                  double y_min, double y_max) {
  assert(boid_to_check >= 0);
  assert(boid_to_check < static_cast<int>(boids.size()));
  Velocity v3{};
  if (neighbours.empty()) {
    return v3;
  }
  Position sum{0.0, 0.0};
  for (int m : neighbours) {
    sum +=
        toroidal_difference(boids[static_cast<std::size_t>(m)].pos,
                            boids[static_cast<std::size_t>(boid_to_check)].pos,
                            x_min, x_max, y_min, y_max);
  }
  int const n = static_cast<int>(neighbours.size());
  Position const cm = ((1.0 / n) * sum);
  Position const v3_pos = c * cm;
  v3 = {v3_pos.x, v3_pos.y};
  return v3;
}

double mean_distance(std::vector<Boid> const& boid, double x_min, double x_max,
                     double y_min, double y_max) {  // distanza media tra boids
  assert(!boid.empty());
  int n = static_cast<int>(boid.size());
  if (n == 1) {
    return 0.0;
  }
  double sum{0.0};
  for (int i = 0; i != n; ++i) {
    for (int j = i + 1; j != n; j++) {
      auto const i_sz = static_cast<std::size_t>(i);
      auto const j_sz = static_cast<std::size_t>(j);
      sum += std::sqrt(toroidal_distance_squared(boid[i_sz].pos, boid[j_sz].pos,
                                                 x_min, x_max, y_min, y_max));
    }
  }
  double const n_pairs = n * (n - 1.0) / 2.0;
  return sum * (1.0 / n_pairs);
}

// disperione che mi dice quando sono divere tra loro le distanze tra coppie,
// vogliamo farla rispetto al centro i massa?
double std_dev_distance(std::vector<Boid> const& boid, double const& mean,
                        double x_min, double x_max, double y_min,
                        double y_max) {
  assert(!boid.empty());
  int n = static_cast<int>(boid.size());
  if (n == 1) {
    return 0.0;
  }
  // double const mean = mean_distance(boid, x_min, x_max, y_min, y_max);
  double sum{0.0};
  for (int i = 0; i != n; ++i) {
    for (int j = i + 1; j != n; j++) {
      auto const i_sz = static_cast<std::size_t>(i);
      auto const j_sz = static_cast<std::size_t>(j);
      double const distance_ij = std::sqrt(toroidal_distance_squared(
          boid[i_sz].pos, boid[j_sz].pos, x_min, x_max, y_min, y_max));
      double const difference = distance_ij - mean;
      sum += difference * difference;
    }
  }
  double const n_pairs = n * (n - 1.0) / 2.0;
  return std::sqrt(sum / n_pairs);
}

double mean_velocity(std::vector<Boid> const& boid) {
  assert(!boid.empty());
  double sum{};
  int n = static_cast<int>(boid.size());
  if (n == 1) {
    return speed_modulus(boid[0].vel);
  }
  for (int i = 0; i != n; ++i) {
    sum += speed_modulus(boid[static_cast<std::size_t>(i)].vel);
  }
  return sum * (1.0 / n);
}

double std_dev_velocity(std::vector<Boid> const& boid, double const& mean) {
  assert(!boid.empty());
  int n = static_cast<int>(boid.size());
  if (n == 1) {
    return 0.0;
  }
  double sum{0.0};
  for (Boid const& m : boid) {
    double const speed = speed_modulus(m.vel);
    double const difference = speed - mean;
    sum += difference * difference;
  }
  return std::sqrt(sum / n);
}

Velocity limit_speed(double v_max, Velocity v_tot, double speed_modulus_) {
  assert(speed_modulus_ > 0.0);
  v_tot.v_x = (v_tot.v_x / speed_modulus_) *
              v_max;  // creo versore modulo 1 e direzione uguale a
                      // v_tot, poi lo moltiplico per v_max così da
                      // avere modulo v_max e direzione v_tot
  v_tot.v_y = (v_tot.v_y / speed_modulus_) * v_max;
  return v_tot;
}

void print(std::vector<Boid> const& boid, double x_min, double x_max,
           double y_min, double y_max) {
  double mean_distance_ = mean_distance(boid, x_min, x_max, y_min, y_max);
  double std_dev_distance_ =
      std_dev_distance(boid, mean_distance_, x_min, x_max, y_min, y_max);
  std::cout << "Distanza media: " << mean_distance_ << " +/- "
            << std_dev_distance_ << '\n';
  double mean_velocity_ = mean_velocity(boid);
  double std_dev_velocity_ = std_dev_velocity(boid, mean_velocity_);
  std::cout << "Velocità media:" << mean_velocity_ << " +/- "
            << std_dev_velocity_ << "\n \n \n";
}

class Flock {
  // valori da mettere in input, inizializzati nel private e definiti col
  // costruttore nel public (così da poter mettere le invarianti e poterli
  // riempire nel main)
 private:
  int n_;     // numero di boids
  double s_;  //<1
  double a_;  //<1
  double c_;  //<1
  double d_;
  double d_s_;          //<d
  double v_max_{10.0};  // double v_max_;
  double
      dt_;  // istante di tempo ogni quanto si aggiornano velocità e posizione
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;

  std::vector<Boid>
      boids_;  // vettore da riempire con il costruttore e generate_n

  // dimensioni schermo in pixel

 public:
  // costruttore, Member Initilisation List
  Flock(int n, double s, double a, double c, double d, double d_s, double dt,
        double x_min, double x_max, double y_min, double y_max)
      : n_{n},
        s_{s},
        a_{a},
        c_{c},
        d_{d},
        d_s_{d_s},
        dt_{dt},
        x_min_{x_min},
        x_max_{x_max},
        y_min_{y_min},
        y_max_{y_max}

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
    if (s_ <= 0 || a_ <= 0 || c_ <= 0 || d_ <= 0 || d_s_ <= 0 || dt_ <= 0) {
      throw std::runtime_error{"Errore: i parametri devono essere positivi"};
    }  // nel main try e catch(std::runtime_error& e){std::cerr << e.what()
       // <<
    // '\n'; return EXIT_FAILURE;}

    boids_ = generate_boid(n, v_max_, x_min, x_max, y_min, y_max);
  }

  // getter per prendere il vettore di boid e usarlo x esempio per media e
  // dev std
  std::vector<Boid> const& boids() const { return boids_; }

  void movement() {
    std::vector<Boid> updatedboids{boids_};
    for (int j = 0; j != n_; ++j) {
      std::vector<int> neighbours =
          neighbours_control(j, d_, boids_, x_min_, x_max_, y_min_, y_max_);
      Velocity v1 = separation(s_, d_s_, j, neighbours, boids_, x_min_, x_max_,
                               y_min_, y_max_);
      Velocity v2 = alignment(a_, j, neighbours, boids_);
      Velocity v3 =
          cohesion(c_, j, neighbours, boids_, x_min_, x_max_, y_min_, y_max_);
      auto const j_sz = static_cast<std::size_t>(j);
      Velocity v_tot = boids_[j_sz].vel + v1 + v2 + v3;
      double speed_modulus_ = speed_modulus(v_tot);
      // condizione velocità massima
      if (v_max_ < speed_modulus_) {
        v_tot = limit_speed(v_max_, v_tot, speed_modulus_);
      }
      updatedboids[j_sz].vel = v_tot;
    }
    for (int j = 0; j != n_; ++j) {
      auto const j_sz = static_cast<std::size_t>(j);
      boids_[j_sz].vel = updatedboids[j_sz].vel;
      Position newp = {boids_[j_sz].pos.x + dt_ * boids_[j_sz].vel.v_x,
                       boids_[j_sz].pos.y + dt_ * boids_[j_sz].vel.v_y};
      // controllo spazio toroidale
      newp = toroidal_space(newp, x_min_, x_max_, y_min_, y_max_);
      boids_[j_sz].pos = newp;
    }
  }
};

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

    Flock prova(n, 0.05, 0.05, 0.005, 100, 20, 1.0, x_min, x_max, y_min,
                y_max);  // oppure da dare in input con txt

    for (int i = 0; i != ngen; ++i) {
      prova.movement();
      print(prova.boids(), x_min, x_max, y_min,
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
