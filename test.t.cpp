#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <stdexcept>

#include "boids.hpp"
#include "doctest.h"
#include "flock.hpp"
#include "statistics.hpp"

// TEST 1: Operatori Vettoriali
// Funzioni testate: operator+ e operator- della struct Position.

//!!SI POSSONO TESTARE GLI ALTRI

TEST_CASE("Testing Vector Operators") {
  pf::Position p1{10.0, 20.0};
  pf::Position p2{5.0, 5.0};

  // Verifica che la sottrazione e la somma componente per componente siano
  // corrette.
  SUBCASE("Position Addition & Subtraction") {
    pf::Position res_sub = p1 - p2;
    CHECK(res_sub.x == doctest::Approx(5.0));
    CHECK(res_sub.y == doctest::Approx(15.0));

    pf::Position res_add = p1 + p2;
    CHECK(res_add.x == doctest::Approx(15.0));
    CHECK(res_add.y == doctest::Approx(25.0));
  }
}

// TEST 2: Geometria Toroidale (gestione dei bordi nello spazio toroidale)
// Funzioni testate: toroidal_space e toroidal_difference.

//!!TESTARE ANCHE OUT SOPRA E SOTTO

TEST_CASE("Testing Toroidal Geometry") {
  pf::Space space{
      0.0,
      100.0,
      0.0,
      100.0,
  };

  // Verifica che un boid che supera un bordo venga teletrasportato al lato
  // opposto.
  SUBCASE("Toroidal Space Wrap-Around") {
    pf::Position out_right{105.0, 50.0};
    pf::Position wrapped_r = pf::toroidal_space(out_right, space);
    CHECK(wrapped_r.x == doctest::Approx(5.0));

    pf::Position out_left{-5.0, 50.0};
    pf::Position wrapped_l = pf::toroidal_space(out_left, space);
    CHECK(wrapped_l.x == doctest::Approx(95.0));
  }

  // Verifica che la distanza calcolata scelga sempre il percorso più breve
  // attraverso i bordi.

  // !!TESTARE ANCHE SOPRA E SOTTO
  SUBCASE("Toroidal Difference (Shortest Distance)") {
    pf::Position p_left{5.0, 50.0};
    pf::Position p_right{95.0, 50.0};
    pf::Position diff = pf::toroidal_difference(p_left, p_right, space);
    CHECK(diff.x == doctest::Approx(10.0));
    CHECK(diff.y == doctest::Approx(0.0));
  }
}

// TEST 3: Rilevamento dei Vicini
// Funzione testata: neighbours_control.

TEST_CASE("Testing Neighbours Detection") {  // TESTARE ANCHE IL CASO NORMALE
  pf::Space space{
      0.0,
      100.0,
      0.0,
      100.0,
  };

  // Verifica che due boid ai lati opposti della mappa si riconoscano
  // come vicini calcolando la distanza minima attraverso il bordo toroidale.
  SUBCASE("Neighbours found across the toroidal border") {
    std::vector<pf::Boid> boids = {{{0, 0}, {98.0, 50.0}},
                                   {{0, 0}, {2.0, 50.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.size() == 1);
    CHECK(neighbours[0] == 1);
  }
}

// TEST 4: Regole di Volo di Reynolds
// Funzioni testate: separation, alignment, cohesion.

TEST_CASE("Testing Flight Rules") {
  pf::Space space{
      0.0,
      100.0,
      0.0,
      100.0,
  };

  // Verifica che la regola induca un allontanamento progressivo dai vicini.
  SUBCASE("Rule 1: Separation (Basic Repulsion)") {
    std::vector<pf::Boid> boids = {{{0, 0}, {10.0, 10.0}},
                                   {{0, 0}, {11.0, 10.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v1 = pf::separation(1.0, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(-1.0));
    CHECK(v1.v_y == doctest::Approx(0.0));
  }

  // Verifica che i boids ignorino la separazione se la loro distanza è maggiore
  // di d_s.
  SUBCASE("Rule 1: Separation ignores neighbours beyond d_s") {
    std::vector<pf::Boid> boids = {{{0, 0}, {10.0, 10.0}},
                                   {{0, 0}, {18.0, 10.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v1 = pf::separation(1.0, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(0.0));
    CHECK(v1.v_y == doctest::Approx(0.0));
  }

  // Verifica che il boid sterzi per allinearsi alla media delle velocità dei
  // compagni.
  SUBCASE("Rule 2: Alignment") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{10.0, 0.0}, {12.0, 10.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v2 = pf::alignment(0.5, 0, neighbours, boids);

    CHECK(v2.v_x == doctest::Approx(5.0));
    CHECK(v2.v_y == doctest::Approx(0.0));
  }

  // Verifica che il boid modifichi la traiettoria per volare verso il centro di
  // massa.
  SUBCASE("Rule 3: Cohesion") {
    std::vector<pf::Boid> boids = {
        {{0, 0}, {0.0, 0.0}}, {{0, 0}, {10.0, 0.0}}, {{0, 0}, {10.0, 10.0}}};
    std::vector<int> neighbours = {1, 2};

    pf::Velocity v3 = pf::cohesion(0.1, 0, neighbours, boids, space);

    CHECK(v3.v_x == doctest::Approx(1.0));
    CHECK(v3.v_y == doctest::Approx(0.5));
  }

  // Verifica il comportamento di sicurezza: se non ci sono vicini, non c'è
  // variazione di velocità.
  SUBCASE("Alignment and cohesion with no neighbours return zero") {
    std::vector<pf::Boid> boids = {{{5.0, 5.0}, {10.0, 10.0}}};
    std::vector<int> neighbours = {};

    pf::Velocity v2 = pf::alignment(0.5, 0, neighbours, boids);
    CHECK(v2.v_x == doctest::Approx(0.0));
    CHECK(v2.v_y == doctest::Approx(0.0));

    pf::Velocity v3 = pf::cohesion(0.1, 0, neighbours, boids, space);
    CHECK(v3.v_x == doctest::Approx(0.0));
    CHECK(v3.v_y == doctest::Approx(0.0));
  }
}

// TEST 5: Limite di Velocità (per garantire l'incontro dei boid)
// Funzioni testate: limit_speed, speed_modulus.

TEST_CASE("Testing Maximum Speed Limit") {
  pf::Velocity v_fast{30.0, 40.0};
  pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, v_fast);

  CHECK(pf::speed_modulus(v_lim) == doctest::Approx(10.0));
  CHECK((v_lim.v_x / v_lim.v_y) == doctest::Approx(30.0 / 40.0));
  CHECK(v_lim.v_x == doctest::Approx(6.0));
  CHECK(v_lim.v_y == doctest::Approx(8.0));
}

TEST_CASE("Testing Minimum Speed Limit") {
  pf::Velocity v_slow{1.2, 1.6};
  pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, v_slow);

  CHECK(pf::speed_modulus(v_lim) == doctest::Approx(3.0));
  CHECK((v_lim.v_x / v_lim.v_y) == doctest::Approx(1.2 / 1.6));
  CHECK(v_lim.v_x == doctest::Approx(1.8));
  CHECK(v_lim.v_y == doctest::Approx(2.4));
}

TEST_CASE("Testing Null Modulus") {
  pf::Velocity v_null{0.0, 0.0};
  pf::Velocity v_lim = pf::limit_speed(3.0, 2.0, v_null);

  CHECK(pf::speed_modulus(v_lim) == doctest::Approx(0.0));
  CHECK(v_lim.v_x == doctest::Approx(0.0));
  CHECK(v_lim.v_y == doctest::Approx(0.0));
}

// TEST 6: Invarianti della Classe (d_s << d per avere lo stormo)
// Funzioni testate: Costruttore principale di Flock.

TEST_CASE("Testing Flock Invariants (Exceptions)") {
  pf::Space space{
      0.0,
      100.0,
      0.0,
      100.0,
  };

  pf::Parameters const valid_par{
      0.05,   // s
      0.05,   // a
      0.005,  // c
      100.0,  // d
      20.0,   // d_s
      3.0,    // v_min
      30.0,   // v_max
      1.0     // dt
  };

  // numero di boids negativo
  CHECK_THROWS_AS(pf::Flock(-5, valid_par, space), std::runtime_error);

  // s negativo
  pf::Parameters par_s = valid_par;
  par_s.s = -0.05;

  CHECK_THROWS_AS(pf::Flock(10, par_s, space), std::invalid_argument);

  // a negativo
  pf::Parameters par_a = valid_par;
  par_a.a = -0.05;

  CHECK_THROWS_AS(pf::Flock(10, par_a, space), std::invalid_argument);

  // c negativo
  pf::Parameters par_c = valid_par;
  par_c.c = -0.005;

  CHECK_THROWS_AS(pf::Flock(10, par_c, space), std::invalid_argument);

  // dt negativo
  pf::Parameters par_dt = valid_par;
  par_dt.dt = -1.0;

  CHECK_THROWS_AS(pf::Flock(10, par_dt, space), std::invalid_argument);
}

// TEST 7: Statistiche dello Stormo
// Funzioni testate: pf::mean_distance, pf::mean_velocity e deviazioni standard.

TEST_CASE("Testing Flock Statistics") {  //!!TESTA ANCHE UN CALCOLO DI DEV STD
                                         //! NON SOLO 0
  pf::Space space{
      0.0,
      100.0,
      0.0,
      100.0,
  };

  // Verifica i calcoli statistici per i moduli delle velocità.
  SUBCASE("Speed mean and standard deviation") {
    std::vector<pf::Boid> boids = {{{3.0, 4.0}, {0.0, 0.0}},
                                   {{3.0, 4.0}, {0.0, 0.0}}};

    double m_vel = pf::mean_velocity(boids);
    CHECK(m_vel == doctest::Approx(5.0));
    CHECK(pf::std_dev_velocity(boids, m_vel) == doctest::Approx(0.0));
  }

  // Verifica i calcoli statistici per le distanze spaziali inter-boid.
  SUBCASE("Distance mean and standard deviation") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {0.0, 0.0}},
                                   {{0.0, 0.0}, {3.0, 4.0}}};

    double m_dist = pf::mean_distance(boids, space);

    CHECK(m_dist == doctest::Approx(5.0).epsilon(0.001));
    CHECK(pf::std_dev_distance(boids, m_dist, space) == doctest::Approx(0.0));
  }

  // Edge case: handling a single boid without division by zero.
  SUBCASE("Edge cases: handling 1 boid without crashing") {
    std::vector<pf::Boid> single_boid = {{{3.0, 4.0}, {10.0, 10.0}}};

    double m_dist = pf::mean_distance(single_boid, space);

    CHECK(m_dist == doctest::Approx(0.0));
    CHECK(pf::std_dev_distance(single_boid, m_dist, space) ==
          doctest::Approx(0.0));

    double m_vel = pf::mean_velocity(single_boid);
    CHECK(m_vel == doctest::Approx(5.0).epsilon(0.001));
    CHECK(pf::std_dev_velocity(single_boid, m_vel) == doctest::Approx(0.0));
  }
}

// TEST 8: Evoluzione Temporale (Delta t)
// Funzione testata: Flock::movement.

TEST_CASE("Testing Time Evolution (Delta t)") {
  pf::Space space{
      0.0,
      100.0,
      0.0,
      100.0,
  };

  pf::Parameters const par{
      0.1,    // s
      0.1,    // a
      0.1,    // c
      200.0,  // d
      5.0,    // d_s
      3.0,    // v_min
      10.0,   // v_max
      1.0     // dt
  };

  pf::Flock flock(2, par, space);

  flock.movement();

  std::vector<pf::Boid> const& boids_after = flock.boids();

  CHECK(boids_after.size() == 2);

  // Verifica che dopo l'evoluzione temporale tutti i boids
  // rimangano all'interno dello spazio toroidale.
  for (pf::Boid const& b : boids_after) {
    CHECK(b.pos.x >= space.x_min);
    CHECK(b.pos.x <= space.x_max);
    CHECK(b.pos.y >= space.y_min);
    CHECK(b.pos.y <= space.y_max);
  }

  // Verifica che la velocità di ogni boid non superi il limite imposto.
  for (pf::Boid const& b : boids_after) {
    CHECK(pf::speed_modulus(b.vel) <= doctest::Approx(10.0));
  }
}

// TEST 9: Generazione dei Boid
// Funzione testata: generate_boid.

TEST_CASE("Testing Boid Generation") {
  pf::Space space{
      0.0,
      100.0,
      0.0,
      100.0,
  };
  double v_max = 10.0;

  std::vector<pf::Boid> boids =
      pf::generate_boid(100, v_max, space);

  CHECK(boids.size() == 100);

  // Verifica che tutte le posizioni generate siano all'interno dello spazio.
  for (pf::Boid const& b : boids) {
    CHECK(b.pos.x >= x_min);
    CHECK(b.pos.x <= x_max);
    CHECK(b.pos.y >= y_min);
    CHECK(b.pos.y <= y_max);

    // Verifica che entrambe le componenti della velocità
    // siano comprese tra -v_max e +v_max.
    CHECK(b.vel.v_x >= -v_max);
    CHECK(b.vel.v_x <= v_max);
    CHECK(b.vel.v_y >= -v_max);
    CHECK(b.vel.v_y <= v_max);
  }
}

// TEST 10: Controllo dei Vicini
// Funzione testata: neighbours_control.

TEST_CASE("Testing Neighbours Control") {
  double x_min = 0.0, x_max = 100.0;
  double y_min = 0.0, y_max = 100.0;

  SUBCASE("The boid does not count itself as a neighbour") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{0.0, 0.0}, {55.0, 50.0}}};

    std::vector<int> neighbours =
        pf::neighbours_control(0, 10.0, boids, x_min, x_max, y_min, y_max);

    CHECK(neighbours.size() == 1);
    CHECK(neighbours[0] == 1);
  }

  SUBCASE("No neighbours are found outside the visual range") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{0.0, 0.0}, {50.0, 50.0}}};

    std::vector<int> neighbours =
        pf::neighbours_control(0, 10.0, boids, x_min, x_max, y_min, y_max);

    CHECK(neighbours.empty());
  }
}

// TEST 11: Casi particolari delle statistiche
// Funzioni testate: mean_distance, std_dev_distance,
// mean_velocity e std_dev_velocity.

/* FORSE NON SERVE! NON DOVREBBE SUCCEDERE CHE I BOID SI SOVRAPPONGANO
TEST_CASE("Testing Statistics with Identical Values") {
  double x_min = 0.0, x_max = 100.0;
  double y_min = 0.0, y_max = 100.0;

  std::vector<pf::Boid> boids = {{{3.0, 4.0}, {10.0, 10.0}},
                                 {{3.0, 4.0}, {10.0, 10.0}},
                                 {{3.0, 4.0}, {10.0, 10.0}}};

  double m_vel = pf::mean_velocity(boids);
  CHECK(m_vel == doctest::Approx(5.0));

  CHECK(pf::std_dev_velocity(boids, m_vel) == doctest::Approx(0.0));

  double m_dist = pf::mean_distance(boids, x_min, x_max, y_min, y_max);

  CHECK(m_dist == doctest::Approx(0.0));

  CHECK(pf::std_dev_distance(boids, m_dist, x_min, x_max, y_min, y_max) ==
        doctest::Approx(0.0));
}
        */