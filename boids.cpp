#include <algorithm>
#include <cassert>
#include <cmath>
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

bool operator==(Position const& a, Position const& b) {
  return {a.x == b.x && a.y == b.y};
}

bool operator!=(Position const& a, Position const& b) { return {!(a == b)}; }


Velocity operator-(Velocity const& a, Velocity const& b) {
  return {a.v_x - b.v_x, a.v_y - b.v_y};
}

Velocity operator+(Velocity const& a, Velocity const& b) {
  return {a.v_x + b.v_x, a.v_y + b.v_y};
}

Velocity operator*(double c, Velocity const& a) {
  return {c * a.v_x, c * a.v_y};
}

double distance(Position const& a, Position const& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

// velocity vector: n entires, each with two coordinates (x, y)
std::vector<Velocity> generate_v(int n) {  // "cin n" in the main
  std::vector<Velocity> velocity;

  std::random_device r;  // seed
  std::default_random_engine eng{r()};
  std::uniform_real_distribution<double> uniform{-3.0, 3.0};  // max velocity?

  std::generate_n(std::back_inserter(velocity), n,
                  [&]() { Velocity v{uniform(eng), uniform(eng)}; });
  std::generate_n(std::back_inserter(velocity), n, [&]() {
    const Velocity v{uniform(eng), uniform(eng)};
    return v;
  });
  return velocity;
}

// positions vector: n entries, each with two coordinates (x, y)
std::vector<Position> generate_p(int n) {  // "cin n" in the main
  std::vector<Position> position;

  std::random_device r;  // seed
  std::default_random_engine eng{r()};
  std::uniform_real_distribution<double> x_uniform{0.0,
                                                   800.0};  // max position?
  std::uniform_real_distribution<double> y_uniform{
      0.0, 600.0};  // grandezze in pixel di uno schermo

  std::generate_n(std::back_inserter(position), n,
                  [&]() { return Position{x_uniform(eng), y_uniform(eng)}; });
  std::generate_n(std::back_inserter(position), n, [&]() {
    const Position p{x_uniform(eng), y_uniform(eng)};
    return p;
  });
  return position;
}

// la funzione neighbours_control vale per un boid solo, ritorna un vettore di
// interi che corrispondono alle posizioni dei boids vicini al boid_to_check nel
// vettore positions (quello con tutti i boids). Si potrebbe dare in input alle
// 3 funzioni delle velocità questo vettore così da non far scorrere tutte e 3
// le volte tutti i boid per trovarne i vicini. Poi (nel main?) si può fare un
// ciclo che fa neighbours_control e le 3 funzioni per le velocità per ogni
// boid. Quindi anche le 3 funzioni per le velocità possono essere rese per un
// singolo boid, per poi fare un ciclo, anche perchè gli n che avevate messo
// dovrebbero essere i numeri di boids vicini che comunque vanno trovati in
// qualche modo direi? Ho provato a fare una roba così.
std::vector<int> neighbours_control(int boid_to_check, double d,
                                    std::vector<Position> const& positions) {
  std::vector<int> neighbours_positions{};
  auto l = static_cast<std::size_t>(boid_to_check);
  for (auto i = 0; i != positions.size(); ++i) {
    if (positions[i] != positions[l] &&
        distance(positions[l], positions[i]) < d) {
      neighbours_positions.push_back(static_cast<int>(i));
    }
  }
  return neighbours_positions;
}


// da rivedere size_t
Velocity separation(double s, double d_s, int boid_to_check,
                    std::vector<int> const& neighbours_control,
                    std::vector<Position> const& positions) {
  Position sum{0.0, 0.0};
  auto l = static_cast<std::size_t>(boid_to_check);
  for (auto i = 0; i != neighbours_control.size(); ++i) {
    auto m = static_cast<std::size_t>(neighbours_control[i]);
    if (distance(positions[l], positions[m]) < d_s) {
      sum = sum +
            (positions[m] - positions[l]);  ////////////algoritmo accumulate?
    }
  }
  Position const pos_v1 = -s * sum;  // perchè const??
  Velocity v1{pos_v1.x, pos_v1.y};
  return v1;
}

Velocity alignment(double a, int boid_to_check,
                   std::vector<int> const& neighbours_control,
                   std::vector<Velocity> const& velocities) {
  Velocity v2{};
  if (neighbours_control.empty()) {
    return v2;
  }
  Velocity sum{0.0, 0.0};
  auto l = static_cast<std::size_t>(boid_to_check);
  for (auto i = 0; i != neighbours_control.size(); ++i) {
    auto m = static_cast<std::size_t>(neighbours_control[i]);
    sum = sum + (velocities[m] - velocities[l]);
  }
  v2 = a *
       (1.0 / (static_cast<int>(neighbours_control.size())) * sum);  // const?

  return v2;
}

Velocity cohesion(double c, int boid_to_check,
                  std::vector<int> const& neighbours_control,
                  std::vector<Position> const& positions) {
  Velocity v3{};
  if (neighbours_control.empty()) {
    return v3;
  }
  Position sum{0.0, 0.0};
  auto l = static_cast<std::size_t>(boid_to_check);
  for (auto i = 0; i != neighbours_control.size(); ++i) {
    auto m = static_cast<std::size_t>(neighbours_control[i]);
    sum = sum + positions[m];
  }
  Position cm = (1.0 / static_cast<int>(neighbours_control.size()) * sum);
  Position v3_pos = c * (cm - positions[l]);
  v3 = {v3_pos.x, v3_pos.y};
  return v3;
}


class Boids {
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
  double dt_;  // istante di tempo ogni quanto si aggiornano velocità e posizione

  std::vector<Velocity> velocities;  // vettori da riempire con il costruttore
  std::vector<Position> positions;

 public:
  // costruttori, Member Initilisation List
  Boids(int n, double s, double a, double c, double d, double d_s, double dt)
      : n_{n}, a_{a}, c_{c}, s_{s}, d_{d}, d_s_{d_s}, dt_{dt}, 
      velocities {generate_v(n)}, positions {generate_p(n)}  { //ora stabiliamo l'invariante di classe con exceptions
        if (n <= 0){
          throw std::runtime_error{"Errore: il numero di boids deve essere maggiore di zero"}; 
        }
        if (d_<=d_s_) {
          throw std::runtime_error{"Errore: il raggio di separazione (d_s) deve essere minore del raggio visivo (d)"};
        } //nel main try e catch(std::runtime_error& e){std::cerr << e.what() << '\n'; return EXIT_FAILURE;}
      }

  auto const print() {
    for (int j = 0; j != n_; ++j) {
      std::cout << j << "° boid: " << '\n';
      std::cout << "Velocity: " << velocities[j].v_x << "," << velocities[j].v_y
                << '\n';
      std::cout << "Position: " << positions[j].x << "," << positions[j].y
                << "\n \n \n";
    }
  }

  void movement() {
    std::vector<Velocity> updatedvelocity{velocities};
    for (int j = 0; j != n_; ++j) {
      std::vector<int> boidvicini = neighbours_control(j, d_, positions);
      if (!boidvicini.empty()) {
        Velocity v1 = separation(s_, d_s_, j, boidvicini, positions);
        Velocity v2 = alignment(a_, j, boidvicini, velocities);
        Velocity v3 = cohesion(c_, j, boidvicini, positions);
        Velocity vtot = velocities[j] + v1 + v2 + v3;
        updatedvelocity[j] = vtot;
      }
    }
    for (int j = 0; j != n_; ++j) {
      velocities[j] = updatedvelocity[j];
      Position newp = {positions[j].x + dt_ * velocities[j].v_x,
                       positions[j].y + dt_ * velocities[j].v_y};

      positions[j] = newp;
    }
    print();
  }
};

int main() {
  int n{};
  int ngen{};
  std::cout << "Quanti boids?" << '\n';
  std::cin >> n;
  std::cout << "Quante iterazioni?" << '\n';
  std::cin >> ngen;
  Boids boid_prova(n, 0.5, 0.5, 0.5, 100, 30, 0.1); //oppure da dare in input con txt
  for (int i = 0; i != ngen; ++i){
    boid_prova.movement();
}
}


