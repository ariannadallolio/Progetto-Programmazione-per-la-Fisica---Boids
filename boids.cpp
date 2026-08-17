#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
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
  std::uniform_real_distribution<double> uniform{0.0, 0.01};  // max velocity?

  std::generate_n(std::back_inserter(velocity), n, [eng, uniform]() mutable {
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
  std::uniform_real_distribution<double> uniform{0.0, 0.2};  // max position?

  std::generate_n(std::back_inserter(position), n, [eng, uniform]() mutable {
    const Position p{uniform(eng), uniform(eng)};
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
                                    std::vector<Position> positions) {
  std::vector<int> neighbours_positions{};
  for (auto i = 0; i != positions.size(); ++i) {
    auto l = static_cast<std::size_t>(boid_to_check);
    if (positions[i] != positions[l] &&
        distance(positions[l], positions[i]) < d) {
      neighbours_positions.push_back(static_cast<int>(i));
    }
  }
  return neighbours_positions;
}

// da rivedere size_t
Velocity separation(double s, double d_s, int boid_to_check,
                    std::vector<int> neighbours_control,
                    std::vector<Position> positions) {
  Position sum{0.0, 0.0};
  for (auto i = 0; i != neighbours_control.size(); ++i) {
    auto l = static_cast<std::size_t>(boid_to_check);
    auto m = static_cast<std::size_t>(neighbours_control[i]);
    if (distance(positions[l], positions[m]) < d_s) {
      sum = sum +
            (positions[l] - positions[m]);  ////////////algoritmo accumulate?
    }
  }
  Position const pos_v1 = -s * sum;  // perchè const??
  Velocity v1{pos_v1.x, pos_v1.y};
  return v1;
}

Velocity alignment(double a, int boid_to_check,
                   std::vector<int> neighbours_control,
                   std::vector<Velocity> velocities) {
  Velocity sum{0.0, 0.0};
  for (auto i = 0; i != neighbours_control.size(); ++i) {
    auto l = static_cast<std::size_t>(boid_to_check);
    auto m = static_cast<std::size_t>(neighbours_control[i]);
    sum = sum + (velocities[l] - velocities[m]);
  }

  Velocity const v2 = a * (1.0 / (static_cast<int>(neighbours_control.size())) *
                           sum);  // const?

  return v2;
}

Velocity cohesion(double c, int boid_to_check,
                  std::vector<int> neighbours_control,
                  std::vector<Position> positions) {
  Position sum{0.0, 0.0};
  auto l = static_cast<std::size_t>(boid_to_check);
  for (auto i = 0; i != neighbours_control.size(); ++i) {
    auto m = static_cast<std::size_t>(neighbours_control[i]);
    sum = sum + positions[m];
  }
  Position cm = (1.0 / static_cast<int>(neighbours_control.size()) * sum);
  Position v3_pos = c * (cm - positions[l]);
  Velocity v3{v3_pos.x, v3_pos.y};
  return v3;
}
// mettere if affinchè divisione non sia 0!

// si potrebbero mettere i valori di input non nella classe ma in un file txt da
// dare in input, e metterci forse vettori positions, velocities, neighbours e i
// 3 metodi per le velocità?
class boid {
  // valori da mettere in input
  double a = 0.2;
  double c = 0.3;
  double d_s = 0.09;
  double d = 0.2;
  double s = 0.6;
  int n = 3;  // numero di boids

  // valori costanti
  int ngen = 10;  // numero di reiterazioni (serve anche per la parte grafica)
  double dt = 0.1;

  // std::vector<Velocity> velocities = generate_v(n);
  // std::vector<Position> positions = generate_p(n);

  /*
  auto movimento() {
    for (int k = 0; k != ngen; ++k) {
      std::vector<Velocity> v1_vector = separation(s, d_s, n, positions);
    }
  }
    */

  auto print(std::vector<Velocity>& velocities,
             std::vector<Position>& positions) {
    for (int j = 0; j != n; ++j) {
      std::cout << "Velocity: " << velocities[j].v_x << "," << velocities[j].v_y
                << '\n';
      std::cout << "Position: " << positions[j].x << "," << positions[j].y
                << '\n';
    }
  }

 public:
  void movement(std::vector<Velocity>& velocities,
                std::vector<Position>& positions) {
    for (int k = 0; k <= ngen; ++k) {
      std::vector<Velocity> updatedvelocity(n);
      for (int j = 0; j != n; ++j) {
        std::vector<int> boidvicini = neighbours_control(j, d, positions);
        Velocity v1 = separation(s, d_s, j, boidvicini, positions);
        Velocity v2 = alignment(a, j, boidvicini, velocities);
        Velocity v3 = cohesion(c, j, boidvicini, positions);
        Velocity vtot = velocities[j] + v1 + v2 + v3;
        updatedvelocity[j] = vtot;
      }
      for (int j = 0; j != n; ++j) {
        velocities[j] = updatedvelocity[j];
        Position newp = {positions[j].x + dt * velocities[j].v_x,
                         positions[j].y + dt * velocities[j].v_y};

        positions[j] = newp;
      }
      print(velocities, positions);
    }
  }
};

int main() {
  int n{};
  std::cout << "Quanti boids?" << '\n';
  std::cin >> n;
  std::vector<Velocity> velocities = generate_v(n);
  std::vector<Position> positions = generate_p(n);

  boid boid_prova{};
  boid_prova.movement(velocities, positions);
}

// vettore posizione e vettore velocità
// calcolo posizione del centro di massa stormo
// ciclo for (su posizoni) per il calcolo di v1 e v3 con assert (distanze
// devono essere minori di d) ciclo for (su velocità) per il calcolo di v2
// con assert (a deve essere <1)

// forse anche una funzione che ogni volta mi trova i boid a distanza <d e
// per le 3 velocità mi usa nei calcoli solo quei boid senza dover scorrerli
// tutti

// per ogni velocità si somma quella "attuale" con v1, v2, v3
// ricalcolo posizioni
