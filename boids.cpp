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

Position operator*(double c, Position const& a) { return {c * a.x, c * a.y}; }


Velocity operator-(Velocity const& a, Velocity const& b) {
  return {a.v_x - b.v_x, a.v_y - b.v_y};
}

Velocity operator+(Velocity const& a, Velocity const& b) {
  return {a.v_x + b.v_x, a.v_y + b.v_y};
}

Velocity operator*(double c, Velocity const& a) { return {c * a.v_x, c * a.v_y}; }


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
  std::uniform_real_distribution<double> uniform{0.0, 1.0};  // max velocity?

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
  std::uniform_real_distribution<double> uniform{0.0, 1.0};  // max position?

  std::generate_n(std::back_inserter(position), n, [eng, uniform]() mutable {
    const Position p{uniform(eng), uniform(eng)};
    return p;
  });
  return position;
}

std::vector<Velocity> separation(double s, double d_s, int n,
                                 std::vector<Position> positions) {
  std::vector<Velocity> v1_sep;
  for (int i = 0; i != n; ++i) {
    Position sum{0.0, 0.0};
    for (int j = 0; j != n; ++j) {
      if (i != j && distance(positions[i], positions[j]) <
                        d_s)  ////////////algoritmo accumulate_if?
        sum = sum + (positions[j] - positions[i]);
    }
    Position const pos_v1 = -s * sum; //perchè const??
    Velocity v1{pos_v1.x, pos_v1.y};
    v1_sep.push_back(v1);
  }
  return v1_sep;
}

std::vector<Velocity> alignment(double a, int n,
                                std::vector<Velocity> velocities) {
 std::vector<Velocity> v2_ali;
 for (int i = 0; i!=n; ++i ) {
 Velocity sum{0.0, 0.0};
 for (int j = 0; i!=n; ++j) {
    if (i!=j) {
        sum = sum + (velocities[j]-velocities[i]);
    }
 }
Velocity const v2 = a * (1/(n-1) * sum);
v2_ali.push_back(v2);
 }
return v2_ali;
}



class boid {
  // valori da mettere in input
  double a;
  double c;
  double d_s;
  double d;
  double s;
  int n;  // numero di boids

  // valori costanti
  int ngen = 100;  // numero di reiterazioni (serve anche per la parte grafica)
  std::vector<Velocity> velocities = generate_v(n);
  std::vector<Position> positions = generate_p(n);

  auto movimento() {
    for (int k = 0; k <= ngen; ++k) {
      std::vector<Velocity> v1_vector = separation(s, d_s, n, positions);
    }
  }
}

  // vettore posizione e vettore velocità
  // calcolo posizione del centro di massa stormo
  // ciclo for (su posizoni) per il calcolo di v1 e v3 con assert (distanze
  // devono essere minori di d) ciclo for (su velocità) per il calcolo di v2 con
  // assert (a deve essere <1)

 //forse anche una funzione che ogni volta mi trova i boid a distanza <d e per le 3 velocità mi usa nei calcoli solo 
 //quei boid senza dover scorrerli tutti 

  // per ogni velocità si somma quella "attuale" con v1, v2, v3
  // ricalcolo posizioni


  //bisogna trovare un modo per trovare i boid a distanza <d
