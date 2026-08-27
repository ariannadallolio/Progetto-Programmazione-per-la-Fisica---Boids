#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <cmath>
#include <stdexcept>
#include <vector>

#include "boids.hpp"
#include "doctest.h"
#include "flock.hpp"
#include "statistics.hpp"

// ===========================================================================
// TEST 1: Operatori della struct Position
// Funzioni testate: operator+, operator-, operator+=, operator* (entrambe le
// forme), operator==, operator!=.
// ===========================================================================

TEST_CASE("Testing Position Operators") {
  pf::Position p1{10.0, 20.0};
  pf::Position p2{5.0, 5.0};

  // Somma e sottrazione componente per componente.
  SUBCASE("Addition & Subtraction") {
    pf::Position res_sub = p1 - p2;
    CHECK(res_sub.x == doctest::Approx(5.0));
    CHECK(res_sub.y == doctest::Approx(15.0));

    pf::Position res_add = p1 + p2;
    CHECK(res_add.x == doctest::Approx(15.0));
    CHECK(res_add.y == doctest::Approx(25.0));
  }

  // Prodotto per scalare: esistono sia (double, Position) sia
  // (Position, double), quindi vanno testate entrambe le overload.
  SUBCASE("Scalar Multiplication (both overloads)") {
    pf::Position p_right = p1 * 2.0;
    CHECK(p_right.x == doctest::Approx(20.0));
    CHECK(p_right.y == doctest::Approx(40.0));

    pf::Position p_left = 0.5 * p1;
    CHECK(p_left.x == doctest::Approx(5.0));
    CHECK(p_left.y == doctest::Approx(10.0));

    // Scalare negativo: e' il caso usato in separation (-s * sum).
    pf::Position p_neg = -1.0 * p1;
    CHECK(p_neg.x == doctest::Approx(-10.0));
    CHECK(p_neg.y == doctest::Approx(-20.0));
  }

  // operator+= modifica l'operando sinistro e restituisce un riferimento ad
  // esso (usato dagli accumulatori in separation e cohesion).
  SUBCASE("Compound Assignment returns reference") {
    pf::Position p_acc{1.0, 2.0};
    pf::Position& ref = (p_acc += p1);

    CHECK(p_acc.x == doctest::Approx(11.0));
    CHECK(p_acc.y == doctest::Approx(22.0));
    CHECK(&ref == &p_acc);
  }

  // Confronto esatto (non Approx): l'operatore usa == sui double.
  SUBCASE("Equality Operators") {
    pf::Position p3{10.0, 20.0};
    pf::Position p4{10.0, 20.001};

    CHECK(p1 == p3);
    CHECK_FALSE(p1 == p4);
    CHECK(p1 != p4);
    CHECK_FALSE(p1 != p3);

    // Differenza su una sola componente alla volta.
    CHECK(pf::Position{1.0, 2.0} != pf::Position{1.0, 3.0});
    CHECK(pf::Position{1.0, 2.0} != pf::Position{9.0, 2.0});
  }
}

// ===========================================================================
// TEST 2: Operatori della struct Velocity
// Funzioni testate: operator+, operator-, operator+=, operator* (entrambe).
// ===========================================================================

TEST_CASE("Testing Velocity Operators") {
  pf::Velocity v1{4.0, 6.0};
  pf::Velocity v2{1.0, 2.0};

  SUBCASE("Addition & Subtraction") {
    pf::Velocity v_add = v1 + v2;
    CHECK(v_add.v_x == doctest::Approx(5.0));
    CHECK(v_add.v_y == doctest::Approx(8.0));

    pf::Velocity v_sub = v1 - v2;
    CHECK(v_sub.v_x == doctest::Approx(3.0));
    CHECK(v_sub.v_y == doctest::Approx(4.0));
  }

  SUBCASE("Compound Assignment") {
    pf::Velocity v_acc{2.0, 2.0};
    v_acc += v1;

    CHECK(v_acc.v_x == doctest::Approx(6.0));
    CHECK(v_acc.v_y == doctest::Approx(8.0));
  }

  SUBCASE("Scalar Multiplication (both overloads)") {
    pf::Velocity v_left = 3.0 * v1;
    CHECK(v_left.v_x == doctest::Approx(12.0));
    CHECK(v_left.v_y == doctest::Approx(18.0));

    pf::Velocity v_right = v1 * 0.5;
    CHECK(v_right.v_x == doctest::Approx(2.0));
    CHECK(v_right.v_y == doctest::Approx(3.0));
  }
}

// ===========================================================================
// TEST 3: Validita' del dominio
// Funzione testata: space_is_valid(Space).
// ===========================================================================

TEST_CASE("Testing Space Validity") {
  CHECK(pf::space_is_valid(pf::Space{0.0, 100.0, 0.0, 100.0}));
  CHECK(pf::space_is_valid(pf::Space{-50.0, 50.0, 100.0, 200.0}));

  // Estremi coincidenti o invertiti: dominio degenere.
  CHECK_FALSE(pf::space_is_valid(pf::Space{0.0, 0.0, 0.0, 100.0}));
  CHECK_FALSE(pf::space_is_valid(pf::Space{0.0, 100.0, 50.0, 50.0}));
  CHECK_FALSE(pf::space_is_valid(pf::Space{100.0, 0.0, 0.0, 100.0}));
  CHECK_FALSE(pf::space_is_valid(pf::Space{0.0, 100.0, 100.0, 0.0}));
}

// ===========================================================================
// TEST 4: Wrap-around dello spazio toroidale
// Funzione testata: toroidal_space.
// ===========================================================================

TEST_CASE("Testing Toroidal Space Wrap-Around") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // Un boid che esce da uno dei 4 bordi rientra dal lato opposto.
  SUBCASE("All 4 borders") {
    pf::Position wrapped_r = pf::toroidal_space({105.0, 50.0}, space);
    CHECK(wrapped_r.x == doctest::Approx(5.0));
    CHECK(wrapped_r.y == doctest::Approx(50.0));

    pf::Position wrapped_l = pf::toroidal_space({-5.0, 50.0}, space);
    CHECK(wrapped_l.x == doctest::Approx(95.0));

    pf::Position wrapped_t = pf::toroidal_space({50.0, 105.0}, space);
    CHECK(wrapped_t.y == doctest::Approx(5.0));

    pf::Position wrapped_b = pf::toroidal_space({50.0, -5.0}, space);
    CHECK(wrapped_b.y == doctest::Approx(95.0));

    // Spostamento superiore a una intera lunghezza del dominio.
    CHECK(pf::toroidal_space({250.0, 50.0}, space).x == doctest::Approx(50.0));
    CHECK(pf::toroidal_space({-150.0, 50.0}, space).x == doctest::Approx(50.0));
    CHECK(pf::toroidal_space({50.0, 320.0}, space).y == doctest::Approx(20.0));
    CHECK(pf::toroidal_space({50.0, -150.0}, space).y == doctest::Approx(50.0));
  }

  // Uscita diagonale: entrambe le componenti vanno corrette.
  SUBCASE("Corner (both components out of range)") {
    pf::Position wrapped = pf::toroidal_space({103.0, -2.0}, space);

    CHECK(wrapped.x == doctest::Approx(3.0));
    CHECK(wrapped.y == doctest::Approx(98.0));
  }

  // Un punto interno non deve essere modificato.
  SUBCASE("Interior point is left unchanged") {
    pf::Position inside{42.0, 17.0};
    pf::Position result = pf::toroidal_space(inside, space);

    CHECK(result == inside);
  }

  // Dominio traslato/negativo: il wrap usa Lx = x_max - x_min, non x_max.
  SUBCASE("Shifted and negative domain") {
    pf::Space shifted{-50.0, 50.0, 100.0, 200.0};

    pf::Position wrapped_x = pf::toroidal_space({55.0, 150.0}, shifted);
    CHECK(wrapped_x.x == doctest::Approx(-45.0));

    pf::Position wrapped_y = pf::toroidal_space({0.0, 95.0}, shifted);
    CHECK(wrapped_y.y == doctest::Approx(195.0));
  }
}

// ===========================================================================
// TEST 5: Distanza minima nello spazio toroidale
// Funzione testata: toroidal_difference.
// ===========================================================================

TEST_CASE("Testing Toroidal Difference") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // Il percorso piu' breve passa attraverso il bordo: 5 - 95 = -90, che
  // supera -Lx/2, quindi viene riportato a +10.
  SUBCASE("Shortest path along X and Y") {
    pf::Position diff_x =
        pf::toroidal_difference({5.0, 50.0}, {95.0, 50.0}, space);
    CHECK(diff_x.x == doctest::Approx(10.0));
    CHECK(diff_x.y == doctest::Approx(0.0));

    pf::Position diff_y =
        pf::toroidal_difference({50.0, 5.0}, {50.0, 95.0}, space);
    CHECK(diff_y.x == doctest::Approx(0.0));
    CHECK(diff_y.y == doctest::Approx(10.0));

    // Caso simmetrico: invertendo gli argomenti cambia solo il segno.
    pf::Position diff_rev =
        pf::toroidal_difference({95.0, 50.0}, {5.0, 50.0}, space);
    CHECK(diff_rev.x == doctest::Approx(-10.0));
  }

  // Se la distanza diretta e' gia' la piu' breve non viene corretta.
  SUBCASE("Direct path is already the shortest") {
    pf::Position diff =
        pf::toroidal_difference({30.0, 60.0}, {20.0, 40.0}, space);

    CHECK(diff.x == doctest::Approx(10.0));
    CHECK(diff.y == doctest::Approx(20.0));
  }

  // Posizioni coincidenti: differenza nulla.
  SUBCASE("Identical positions") {
    pf::Position p{25.0, 75.0};
    pf::Position diff = pf::toroidal_difference(p, p, space);

    CHECK(diff.x == doctest::Approx(0.0));
    CHECK(diff.y == doctest::Approx(0.0));
  }

  // Esattamente a Lx/2 le due direzioni sono equivalenti: la condizione e'
  // una disuguaglianza stretta, quindi il valore resta -50 e non viene
  // riportato a +50. Si controlla il modulo, non il segno.
  SUBCASE("Mid-space boundary (exactly Lx / 2)") {
    pf::Position diff =
        pf::toroidal_difference({0.0, 50.0}, {50.0, 50.0}, space);

    CHECK(std::abs(diff.x) == doctest::Approx(50.0));
  }
  SUBCASE("Mid-space boundary (exactly Ly / 2)") {
    pf::Position diff =
        pf::toroidal_difference({50.0, 0.0}, {50.0, 50.0}, space);

    CHECK(std::abs(diff.y) == doctest::Approx(50.0));
  }
}

// ===========================================================================
// TEST 6: Distanza quadrata toroidale e modulo della velocita'
// Funzioni testate: toroidal_distance_squared, speed_modulus.
// ===========================================================================

TEST_CASE("Testing Toroidal Distance Squared and Speed Modulus") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  SUBCASE("Distance squared, direct and across the border") {
    // Triangolo 3-4-5: distanza 5, quadrato 25.
    CHECK(pf::toroidal_distance_squared({0.0, 0.0}, {3.0, 4.0}, space) ==
          doctest::Approx(25.0));

    // Attraverso il bordo: dx = 2, dy = 0 -> 4.
    CHECK(pf::toroidal_distance_squared({99.0, 50.0}, {1.0, 50.0}, space) ==
          doctest::Approx(4.0));

    // Simmetria rispetto allo scambio degli argomenti.
    CHECK(pf::toroidal_distance_squared({10.0, 20.0}, {40.0, 60.0}, space) ==
          doctest::Approx(pf::toroidal_distance_squared({40.0, 60.0},
                                                        {10.0, 20.0}, space)));

    // Distanza di un punto da se' stesso.
    CHECK(pf::toroidal_distance_squared({10.0, 20.0}, {10.0, 20.0}, space) ==
          doctest::Approx(0.0));
  }

  SUBCASE("Speed modulus") {
    CHECK(pf::speed_modulus({3.0, 4.0}) == doctest::Approx(5.0));
    CHECK(pf::speed_modulus({-3.0, -4.0}) == doctest::Approx(5.0));
    CHECK(pf::speed_modulus({0.0, 0.0}) == doctest::Approx(0.0));
    CHECK(pf::speed_modulus({0.0, -7.0}) == doctest::Approx(7.0));
  }
}

// ===========================================================================
// TEST 7: Rilevamento dei vicini
// Funzione testata: neighbours_control.
// ===========================================================================

TEST_CASE("Testing Neighbours Detection and Control") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // Caso standard: solo il boid entro il raggio d viene selezionato.
  SUBCASE("Neighbours found in standard Euclidean distance") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{0.0, 0.0}, {53.0, 54.0}},
                                   {{0.0, 0.0}, {80.0, 80.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.size() == 1);
    CHECK(neighbours[0] == 1);
  }

  // Due boid ai lati opposti della mappa si riconoscono come vicini grazie
  // alla distanza toroidale.
  SUBCASE("Neighbours found across the toroidal border") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {98.0, 50.0}},
                                   {{0.0, 0.0}, {2.0, 50.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.size() == 1);
    CHECK(neighbours[0] == 1);
  }

  // Nessun vicino oltre il raggio visivo.
  SUBCASE("No neighbours are found outside the visual range") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{0.0, 0.0}, {50.0, 50.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.empty());
  }

  // Il confronto e' una disuguaglianza stretta: a distanza esattamente d il
  // boid NON viene contato come vicino.
  SUBCASE("Distance exactly equal to d is excluded") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{0.0, 0.0}, {60.0, 50.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.empty());
  }

  // Piu' vicini: gli indici sono restituiti in ordine crescente ed
  // escludono quello fuori raggio.
  SUBCASE("Multiple neighbours, indices in increasing order") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{0.0, 0.0}, {52.0, 50.0}},
                                   {{0.0, 0.0}, {90.0, 90.0}},
                                   {{0.0, 0.0}, {50.0, 47.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.size() == 2);
    CHECK(neighbours[0] == 1);
    CHECK(neighbours[1] == 3);
  }

  // Un solo boid nello stormo: nessun vicino possibile.
  SUBCASE("Single boid has no neighbours") {
    std::vector<pf::Boid> boids = {{{1.0, 1.0}, {50.0, 50.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.empty());
  }
}

// ===========================================================================
// TEST 8: Regola 1 di Reynolds - Separazione
// Funzione testata: separation.
// ===========================================================================

TEST_CASE("Testing Rule 1: Separation") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // v1 = -s * somma(pos_vicino - pos_boid): il boid viene spinto in
  // direzione opposta al vicino.
  SUBCASE("TBasic repulsion") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{0.0, 0.0}, {12.0, 14.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v1 = pf::separation(0.5, 10.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(-1.0));
    CHECK(v1.v_y == doctest::Approx(-2.0));
  }

  // I vicini oltre d_s non contribuiscono (distanza 8 > d_s = 5).
  SUBCASE("Neighbours beyond d_s are ignored") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{0.0, 0.0}, {18.0, 10.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v1 = pf::separation(0.5, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(0.0));
    CHECK(v1.v_y == doctest::Approx(0.0));
  }

  // Contributi di piu' vicini che si sommano: uno a destra e uno sopra.
  SUBCASE("Contributions of several neighbours are summed") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{0.0, 0.0}, {52.0, 50.0}},
                                   {{0.0, 0.0}, {50.0, 53.0}}};
    std::vector<int> neighbours = {1, 2};

    pf::Velocity v1 = pf::separation(1.0, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(-2.0));
    CHECK(v1.v_y == doctest::Approx(-3.0));
  }

  // La repulsione usa la distanza toroidale: il vicino "oltre il bordo"
  // spinge verso l'interno del dominio.
  SUBCASE("Repulsion across the toroidal border") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {99.0, 50.0}},
                                   {{0.0, 0.0}, {1.0, 50.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v1 = pf::separation(1.0, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(-2.0));
    CHECK(v1.v_y == doctest::Approx(0.0));
  }

  // Nessun vicino: nessuna variazione di velocita'.
  SUBCASE("No neighbours returns zero") {
    std::vector<pf::Boid> boids = {{{5.0, 5.0}, {10.0, 10.0}}};
    std::vector<int> neighbours = {};

    pf::Velocity v1 = pf::separation(1.0, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(0.0));
    CHECK(v1.v_y == doctest::Approx(0.0));
  }
}

// ===========================================================================
// TEST 9: Regola 2 di Reynolds - Allineamento
// Funzione testata: alignment.
// ===========================================================================

TEST_CASE("Testing Rule 2: Alignment") {
  // v2 = a * media(v_vicino - v_boid) = 0.5 * (10 - 0) = 5.
  SUBCASE("Steering towards the neighbours' mean velocity") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{10.0, 0.0}, {12.0, 10.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v2 = pf::alignment(0.5, 0, neighbours, boids);

    CHECK(v2.v_x == doctest::Approx(5.0));
    CHECK(v2.v_y == doctest::Approx(0.0));
  }

  // Media su due vicini: ((4,0) + (0,8)) / 2 = (2,4), scalato per a = 0.5.
  SUBCASE("Mean over several neighbours") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{4.0, 0.0}, {52.0, 50.0}},
                                   {{0.0, 8.0}, {50.0, 52.0}}};
    std::vector<int> neighbours = {1, 2};

    pf::Velocity v2 = pf::alignment(0.5, 0, neighbours, boids);

    CHECK(v2.v_x == doctest::Approx(1.0));
    CHECK(v2.v_y == doctest::Approx(2.0));
  }

  // Se il boid ha gia' la stessa velocita' dei vicini non deve sterzare.
  SUBCASE("Already aligned boid gets no correction") {
    std::vector<pf::Boid> boids = {{{3.0, 4.0}, {50.0, 50.0}},
                                   {{3.0, 4.0}, {52.0, 50.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v2 = pf::alignment(0.5, 0, neighbours, boids);

    CHECK(v2.v_x == doctest::Approx(0.0));
    CHECK(v2.v_y == doctest::Approx(0.0));
  }

  SUBCASE("No neighbours returns zero") {
    std::vector<pf::Boid> boids = {{{5.0, 5.0}, {10.0, 10.0}}};
    std::vector<int> neighbours = {};

    pf::Velocity v2 = pf::alignment(0.5, 0, neighbours, boids);

    CHECK(v2.v_x == doctest::Approx(0.0));
    CHECK(v2.v_y == doctest::Approx(0.0));
  }
}

// ===========================================================================
// TEST 10: Regola 3 di Reynolds - Coesione
// Funzione testata: cohesion.
// ===========================================================================

TEST_CASE("Testing Rule 3: Cohesion") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // v3 = c * (centro di massa dei vicini - posizione del boid):
  // ((10,0) + (10,10)) / 2 = (10,5), scalato per c = 0.1.
  SUBCASE("Steering towards the centre of mass") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {0.0, 0.0}},
                                   {{0.0, 0.0}, {10.0, 0.0}},
                                   {{0.0, 0.0}, {10.0, 10.0}}};
    std::vector<int> neighbours = {1, 2};

    pf::Velocity v3 = pf::cohesion(0.1, 0, neighbours, boids, space);

    CHECK(v3.v_x == doctest::Approx(1.0));
    CHECK(v3.v_y == doctest::Approx(0.5));
  }

  // Il centro di massa usa le differenze toroidali: il boid a x = 99 deve
  // essere attratto verso destra (oltre il bordo), non verso sinistra.
  SUBCASE("Centre of mass across the toroidal border") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {99.0, 50.0}},
                                   {{0.0, 0.0}, {1.0, 50.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v3 = pf::cohesion(0.5, 0, neighbours, boids, space);

    CHECK(v3.v_x == doctest::Approx(1.0));
    CHECK(v3.v_y == doctest::Approx(0.0));
  }

  // Boid gia' nel centro di massa dei vicini: nessuna correzione.
  SUBCASE("Boid already at the centre of mass") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{0.0, 0.0}, {40.0, 50.0}},
                                   {{0.0, 0.0}, {60.0, 50.0}}};
    std::vector<int> neighbours = {1, 2};

    pf::Velocity v3 = pf::cohesion(0.1, 0, neighbours, boids, space);

    CHECK(v3.v_x == doctest::Approx(0.0));
    CHECK(v3.v_y == doctest::Approx(0.0));
  }

  SUBCASE("No neighbours returns zero") {
    std::vector<pf::Boid> boids = {{{5.0, 5.0}, {10.0, 10.0}}};
    std::vector<int> neighbours = {};

    pf::Velocity v3 = pf::cohesion(0.1, 0, neighbours, boids, space);

    CHECK(v3.v_x == doctest::Approx(0.0));
    CHECK(v3.v_y == doctest::Approx(0.0));
  }
}

// ===========================================================================
// TEST 11: Limiti di velocita'
// Funzione testata: limit_speed (con speed_modulus).
// ===========================================================================

TEST_CASE("Testing Speed Limits") {
  // Sopra v_max: il modulo viene riportato a v_max mantenendo la direzione.
  SUBCASE("Maximum speed limit") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {30.0, 40.0});

    CHECK(pf::speed_modulus(v_lim) == doctest::Approx(10.0));
    CHECK((v_lim.v_x / v_lim.v_y) == doctest::Approx(30.0 / 40.0));
    CHECK(v_lim.v_x == doctest::Approx(6.0));
    CHECK(v_lim.v_y == doctest::Approx(8.0));
  }

  // Sotto v_min: il modulo viene portato a v_min, direzione invariata.
  SUBCASE("Minimum speed limit") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {1.2, 1.6});

    CHECK(pf::speed_modulus(v_lim) == doctest::Approx(3.0));
    CHECK((v_lim.v_x / v_lim.v_y) == doctest::Approx(1.2 / 1.6));
    CHECK(v_lim.v_x == doctest::Approx(1.8));
    CHECK(v_lim.v_y == doctest::Approx(2.4));
  }

  // Velocita' gia' nel range: nessuna modifica.
  SUBCASE("Speed already inside the range is unchanged") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {3.0, 4.0});

    CHECK(v_lim.v_x == doctest::Approx(3.0));
    CHECK(v_lim.v_y == doctest::Approx(4.0));
  }

  // La direzione (segno delle componenti) e' preservata anche nel terzo
  // quadrante.
  SUBCASE("Direction is preserved for negative components") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {-30.0, -40.0});

    CHECK(v_lim.v_x == doctest::Approx(-6.0));
    CHECK(v_lim.v_y == doctest::Approx(-8.0));
  }

  // Modulo nullo: non e' normalizzabile, la funzione deve restituire il
  // vettore invariato senza dividere per zero (niente NaN).
  SUBCASE("Null modulus does not produce NaN") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {0.0, 0.0});

    CHECK(pf::speed_modulus(v_lim) == doctest::Approx(0.0));
    CHECK(v_lim.v_x == doctest::Approx(0.0));
    CHECK(v_lim.v_y == doctest::Approx(0.0));
  }
}

// ===========================================================================
// TEST 12: Generazione casuale dei boid
// Funzione testata: generate_boid.
// ===========================================================================

TEST_CASE("Testing Boid Generation") {
  // La generazione estrae un modulo uniforme in [v_min, v_max] e un angolo
  // uniforme in [0, 2pi): il vincolo e' sul modulo, non sulle componenti.
  SUBCASE("Positions inside the domain and modulus inside [v_min, v_max]") {
    pf::Space space{0.0, 100.0, 0.0, 100.0};
    double const v_min = 3.0;
    double const v_max = 10.0;

    std::vector<pf::Boid> boids = pf::generate_boid(100, v_min, v_max, space);

    CHECK(boids.size() == 100);

    for (pf::Boid const& b : boids) {
      CHECK(b.pos.x >= space.x_min);
      CHECK(b.pos.x <= space.x_max);
      CHECK(b.pos.y >= space.y_min);
      CHECK(b.pos.y <= space.y_max);

      CHECK(pf::speed_modulus(b.vel) >= doctest::Approx(v_min));
      CHECK(pf::speed_modulus(b.vel) <= doctest::Approx(v_max));
    }
  }

  // Dominio traslato e con coordinate negative: le distribuzioni devono
  // usare gli estremi effettivi, non [0, x_max].
  SUBCASE("Shifted and negative rectangular domain") {
    pf::Space space{-50.0, 50.0, 100.0, 200.0};
    double const v_min = 1.0;
    double const v_max = 5.0;

    std::vector<pf::Boid> boids = pf::generate_boid(50, v_min, v_max, space);

    CHECK(boids.size() == 50);

    for (pf::Boid const& b : boids) {
      CHECK(b.pos.x >= -50.0);
      CHECK(b.pos.x <= 50.0);
      CHECK(b.pos.y >= 100.0);
      CHECK(b.pos.y <= 200.0);

      CHECK(pf::speed_modulus(b.vel) >= doctest::Approx(v_min));
      CHECK(pf::speed_modulus(b.vel) <= doctest::Approx(v_max));
    }
  }

  // v_min == v_max: tutti i boid hanno lo stesso modulo, direzioni diverse.
  SUBCASE("Degenerate speed range (v_min == v_max)") {
    pf::Space space{0.0, 100.0, 0.0, 100.0};

    std::vector<pf::Boid> boids = pf::generate_boid(20, 5.0, 5.0, space);

    for (pf::Boid const& b : boids) {
      CHECK(pf::speed_modulus(b.vel) == doctest::Approx(5.0));
    }
  }
}

// ===========================================================================
// TEST 13: Costruzione valida di Flock e getter
// Funzioni testate: Flock::Flock, Flock::boids, Flock::parameters,
// Flock::space.
// ===========================================================================

TEST_CASE("Testing Flock Construction and Getters") {
  pf::Space space{0.0, 800.0, 0.0, 600.0};

  pf::Parameters const par{
      100,    // n
      0.05,   // s
      0.05,   // a
      0.005,  // c
      100.0,  // d
      20.0,   // d_s
      3.0,    // v_min
      30.0,   // v_max
      1.0     // dt
  };

  pf::Flock flock(par, space);

  // Il costruttore deve popolare lo stormo con esattamente n boid.
  CHECK(flock.boids().size() == 25);

  // I getter restituiscono i parametri e il dominio passati in input.
  CHECK(flock.parameters().n_boids == doctest::Approx(par.n_boids));
  CHECK(flock.parameters().s == doctest::Approx(par.s));
  CHECK(flock.parameters().a == doctest::Approx(par.a));
  CHECK(flock.parameters().c == doctest::Approx(par.c));
  CHECK(flock.parameters().d == doctest::Approx(par.d));
  CHECK(flock.parameters().d_s == doctest::Approx(par.d_s));
  CHECK(flock.parameters().v_min == doctest::Approx(par.v_min));
  CHECK(flock.parameters().v_max == doctest::Approx(par.v_max));
  CHECK(flock.parameters().dt == doctest::Approx(par.dt));
  CHECK(flock.space().x_min == doctest::Approx(space.x_min));
  CHECK(flock.space().x_max == doctest::Approx(space.x_max));
  CHECK(flock.space().y_min == doctest::Approx(space.y_min));
  CHECK(flock.space().y_max == doctest::Approx(space.y_max));
}

// ===========================================================================
// TEST 14: Invarianti della classe Flock
// Funzioni testate: Flock::Flock e check_parameters (eccezioni).
// ===========================================================================

TEST_CASE("Testing Flock Invariants (Exceptions)") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  pf::Parameters const valid_par{
      100,    // n
      0.05,   // s
      0.05,   // a
      0.005,  // c
      100.0,  // d
      20.0,   // d_s
      3.0,    // v_min
      30.0,   // v_max
      1.0     // dt
  };

  // Controllo di riferimento: con parametri validi non si lancia nulla.
  CHECK_NOTHROW(pf::Flock(valid_par, space));

  // n <= 0 -> runtime_error (verificato prima di check_parameters).
  SUBCASE("Invalid number of boids") {
    CHECK_THROWS_AS(pf::Flock(valid_par, space), std::invalid_argument);
    CHECK_THROWS_AS(pf::Flock(valid_par, space), std::invalid_argument);
  }

  // Dominio degenere.
  SUBCASE("Invalid space") {
    CHECK_THROWS_AS(pf::Flock(valid_par, pf::Space{0.0, 0.0, 0.0, 100.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(pf::Flock(valid_par, pf::Space{0.0, 100.0, 50.0, 20.0}),
                    std::invalid_argument);
  }

  // I fattori s, a, c devono essere strettamente positivi.
  SUBCASE("Non-positive s, a, c") {
    pf::Parameters par_s = valid_par;
    par_s.s = -0.05;
    CHECK_THROWS_AS(pf::Flock(par_s, space), std::invalid_argument);

    pf::Parameters par_s0 = valid_par;
    par_s0.s = 0.0;
    CHECK_THROWS_AS(pf::Flock(par_s0, space), std::invalid_argument);

    pf::Parameters par_a = valid_par;
    par_a.a = -0.05;
    CHECK_THROWS_AS(pf::Flock(par_a, space), std::invalid_argument);

    pf::Parameters par_c = valid_par;
    par_c.c = -0.005;
    CHECK_THROWS_AS(pf::Flock(par_c, space), std::invalid_argument);
  }

  // I raggi devono essere positivi.
  SUBCASE("Non-positive radii") {
    pf::Parameters par_d = valid_par;
    par_d.d = -100.0;
    CHECK_THROWS_AS(pf::Flock(par_d, space), std::invalid_argument);

    pf::Parameters par_ds = valid_par;
    par_ds.d_s = 0.0;
    CHECK_THROWS_AS(pf::Flock(par_ds, space), std::invalid_argument);
  }

  // Invariante fondamentale del modello: d_s < d.
  SUBCASE("Separation radius must be smaller than perception radius") {
    pf::Parameters par_eq = valid_par;
    par_eq.d = 20.0;  // d == d_s
    CHECK_THROWS_AS(pf::Flock(par_eq, space), std::invalid_argument);

    pf::Parameters par_lt = valid_par;
    par_lt.d = 10.0;  // d < d_s
    CHECK_THROWS_AS(pf::Flock(par_lt, space), std::invalid_argument);
  }

  // Vincoli sulle velocita': v_max > 0, v_min >= 0, v_min < v_max.
  SUBCASE("Invalid speed limits") {
    pf::Parameters par_vmax = valid_par;
    par_vmax.v_max = 0.0;
    CHECK_THROWS_AS(pf::Flock(par_vmax, space), std::invalid_argument);

    pf::Parameters par_vmin = valid_par;
    par_vmin.v_min = -1.0;
    CHECK_THROWS_AS(pf::Flock(par_vmin, space), std::invalid_argument);

    pf::Parameters par_veq = valid_par;
    par_veq.v_min = 30.0;  // v_min == v_max
    CHECK_THROWS_AS(pf::Flock(par_veq, space), std::invalid_argument);

    pf::Parameters par_vinv = valid_par;
    par_vinv.v_min = 40.0;  // v_min > v_max
    CHECK_THROWS_AS(pf::Flock(par_vinv, space), std::invalid_argument);

    // v_min = 0 e' invece ammesso dalle invarianti.
    pf::Parameters par_vzero = valid_par;
    par_vzero.v_min = 0.0;
    CHECK_NOTHROW(pf::Flock(par_vzero, space));
  }

  // Il passo temporale deve essere positivo.
  SUBCASE("Non-positive dt") {
    pf::Parameters par_dt = valid_par;
    par_dt.dt = -1.0;
    CHECK_THROWS_AS(pf::Flock(par_dt, space), std::invalid_argument);

    pf::Parameters par_dt0 = valid_par;
    par_dt0.dt = 0.0;
    CHECK_THROWS_AS(pf::Flock(par_dt0, space), std::invalid_argument);
  }
}

// ===========================================================================
// TEST 15: Evoluzione temporale
// Funzione testata: Flock::movement.
// ===========================================================================

TEST_CASE("Testing Time Evolution (movement)") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  pf::Parameters const par{
      100,   // n
      0.1,   // s
      0.1,   // a
      0.1,   // c
      50.0,  // d
      5.0,   // d_s
      3.0,   // v_min
      10.0,  // v_max
      1.0    // dt
  };

  // Dopo l'aggiornamento i boid restano nel dominio e nei limiti di
  // velocita', e il loro numero non cambia. Si itera piu' volte per non
  // testare solo il primo passo.
  SUBCASE("Movement preserves flock constraints") {
    pf::Flock flock(par, space);

    for (int step = 0; step != 10; ++step) {
      flock.movement();
    }

    std::vector<pf::Boid> const& boids_after = flock.boids();

    CHECK(boids_after.size() == 20);

    for (pf::Boid const& b : boids_after) {
      CHECK(b.pos.x >= space.x_min);
      CHECK(b.pos.x <= space.x_max);
      CHECK(b.pos.y >= space.y_min);
      CHECK(b.pos.y <= space.y_max);

      CHECK(pf::speed_modulus(b.vel) <= doctest::Approx(par.v_max));
      CHECK(pf::speed_modulus(b.vel) >= doctest::Approx(par.v_min));
    }
  }

  // Verifica esplicita dell'integrazione:
  // pos_new = toroidal_space(pos_old + dt * v_new).
  SUBCASE("Positions are updated with dt and the new velocity") {
    pf::Flock flock(par, space);

    std::vector<pf::Boid> const before = flock.boids();  // copia

    flock.movement();

    std::vector<pf::Boid> const& after = flock.boids();

    for (std::size_t i = 0; i != after.size(); ++i) {
      pf::Position const expected =
          pf::toroidal_space({before[i].pos.x + par.dt * after[i].vel.v_x,
                              before[i].pos.y + par.dt * after[i].vel.v_y},
                             space);

      CHECK(after[i].pos.x == doctest::Approx(expected.x));
      CHECK(after[i].pos.y == doctest::Approx(expected.y));
    }
  }

  // Un boid isolato non ha vicini: le tre regole danno contributo nullo e il
  // modulo generato e' gia' nel range, quindi la velocita' non cambia.
  SUBCASE("A single boid keeps its velocity") {
    pf::Flock flock(par, space);

    pf::Velocity const v_before = flock.boids()[0].vel;

    flock.movement();

    pf::Velocity const v_after = flock.boids()[0].vel;

    CHECK(v_after.v_x == doctest::Approx(v_before.v_x));
    CHECK(v_after.v_y == doctest::Approx(v_before.v_y));
  }
}

// ===========================================================================
// TEST 16: Statistiche sulle velocita'
// Funzioni testate: mean_velocity, std_dev_velocity.
// NB: la deviazione standard implementata e' quella di popolazione
// (divisione per N, non per N-1).
// ===========================================================================

TEST_CASE("Testing Velocity Statistics") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};
  // Moduli identici -> media pari al modulo, deviazione nulla.
  SUBCASE("Constant modulus gives zero standard deviation") {
    std::vector<pf::Boid> boids = {{{3.0, 4.0}, {0.0, 0.0}},
                                   {{3.0, 4.0}, {0.0, 0.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_vel = stats.mean_velocity;

    CHECK(m_vel == doctest::Approx(5.0));
    CHECK(stats.std_dev_velocity == doctest::Approx(0.0));
  }
  // Moduli 5 e 10 -> media 7.5; scarti +/- 2.5 -> sqrt(12.5 / 2) = 2.5.
  SUBCASE("Non-zero standard deviation (two boids)") {
    std::vector<pf::Boid> boids = {{{3.0, 4.0}, {0.0, 0.0}},
                                   {{6.0, 8.0}, {10.0, 0.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_vel = stats.mean_velocity;

    CHECK(m_vel == doctest::Approx(7.5));
    CHECK(stats.std_dev_velocity == doctest::Approx(2.5));
  }

  // Moduli 3, 5, 7 -> media 5; scarti -2, 0, +2 -> sqrt(8 / 3) = 1.632993.
  SUBCASE("Non-zero standard deviation (three boids)") {
    std::vector<pf::Boid> boids = {{{3.0, 0.0}, {0.0, 0.0}},
                                   {{0.0, 5.0}, {10.0, 0.0}},
                                   {{0.0, -7.0}, {20.0, 0.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_vel = stats.mean_velocity;

    CHECK(m_vel == doctest::Approx(5.0));
    CHECK(stats.std_dev_velocity == doctest::Approx(1.632993).epsilon(0.001));
  }

  // Il modulo e' sempre positivo: velocita' opposte danno la stessa media.
  SUBCASE("Opposite velocities have the same modulus") {
    std::vector<pf::Boid> boids = {{{3.0, 4.0}, {0.0, 0.0}},
                                   {{-3.0, -4.0}, {10.0, 0.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_vel = stats.mean_velocity;

    CHECK(m_vel == doctest::Approx(5.0));
    CHECK(stats.std_dev_velocity == doctest::Approx(0.0));
  }
}

// ===========================================================================
// TEST 17: Statistiche sulle distanze
// Funzioni testate: mean_distance, std_dev_distance.
// La media e' calcolata su tutte le n(n-1)/2 coppie distinte.
// ===========================================================================

TEST_CASE("Testing Distance Statistics") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // Due boid, una sola coppia: media = distanza, deviazione nulla.
  SUBCASE("Two boids: mean equals the distance") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {0.0, 0.0}},
                                   {{0.0, 0.0}, {3.0, 4.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_dist = stats.mean_distance;

    CHECK(m_dist == doctest::Approx(5.0).epsilon(0.001));
    CHECK(stats.std_dev_distance == doctest::Approx(0.0));
  }

  // Le distanze sono toroidali: 1 e 99 distano 2, non 98.
  SUBCASE("Distances are computed on the torus") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {1.0, 50.0}},
                                   {{0.0, 0.0}, {99.0, 50.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_dist = stats.mean_distance;

    CHECK(m_dist == doctest::Approx(2.0));
  }

  // Tre boid ai vertici di un triangolo rettangolo 6-8-10: le coppie
  // valgono 6, 8, 10 -> media 8; scarti -2, 0, +2 -> sqrt(8 / 3).
  SUBCASE("Three boids: non-zero standard deviation") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{0.0, 0.0}, {16.0, 10.0}},
                                   {{0.0, 0.0}, {10.0, 18.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_dist = stats.mean_distance;

    CHECK(m_dist == doctest::Approx(8.0));
    CHECK(stats.std_dev_distance == doctest::Approx(1.632993).epsilon(0.001));
  }
}

// ===========================================================================
// TEST 18: Casi limite delle statistiche
// Un solo boid: nessuna coppia, quindi nessuna divisione per zero.
// ===========================================================================

TEST_CASE("Testing Statistics Edge Cases (1 boid)") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  std::vector<pf::Boid> single_boid = {{{3.0, 4.0}, {10.0, 10.0}}};

  pf::Statistics stats = pf::statistics(single_boid, space);
  double const m_dist = stats.mean_distance;

  CHECK(m_dist == doctest::Approx(0.0));
  CHECK(stats.std_dev_distance == doctest::Approx(0.0));

  // Con un solo boid la media delle velocita' e' il suo stesso modulo.
  double const m_vel = stats.mean_velocity;

  CHECK(m_vel == doctest::Approx(5.0));
  CHECK(stats.std_dev_velocity == doctest::Approx(0.0));
}