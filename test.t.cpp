#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <cmath>
#include <stdexcept>
#include <vector>

#include "boids.hpp"
#include "doctest.h"
#include <cstdio>
#include <fstream>

#include "flock.hpp"
#include "input_parameters.hpp"
#include "statistics.hpp"

// TEST 1: Position struct operators
// Tested functions: operator+, operator-, operator+=, operator* (both forms), operator==, operator!=.

TEST_CASE("Testing Position Operators") {
  pf::Position p1{10.0, 20.0};
  pf::Position p2{5.0, 5.0};

  // Component-wise addition and subtraction.
  SUBCASE("Addition & Subtraction") {
    pf::Position res_sub = p1 - p2;
    CHECK(res_sub.x == doctest::Approx(5.0));
    CHECK(res_sub.y == doctest::Approx(15.0));

    pf::Position res_add = p1 + p2;
    CHECK(res_add.x == doctest::Approx(15.0));
    CHECK(res_add.y == doctest::Approx(25.0));
  }

  // Scalar multiplication:(double, Position) and (Position, double)
  SUBCASE("Scalar Multiplication (both overloads)") {
    pf::Position p_right = p1 * 2.0;
    CHECK(p_right.x == doctest::Approx(20.0));
    CHECK(p_right.y == doctest::Approx(40.0));

    pf::Position p_left = 0.5 * p1;
    CHECK(p_left.x == doctest::Approx(5.0));
    CHECK(p_left.y == doctest::Approx(10.0));

    // Negative scalar: this case is used in separation (-s * sum).
    pf::Position p_neg = -1.0 * p1;
    CHECK(p_neg.x == doctest::Approx(-10.0));
    CHECK(p_neg.y == doctest::Approx(-20.0));
  }

  // operator+= modifies the left operand and returns a reference to it
  // (used by accumulators in separation and cohesion).
  SUBCASE("Compound Assignment returns reference") {
    pf::Position p_acc{1.0, 2.0};
    pf::Position& ref = (p_acc += p1);

    CHECK(p_acc.x == doctest::Approx(11.0));
    CHECK(p_acc.y == doctest::Approx(22.0));
    CHECK(&ref == &p_acc);
  }

  // Exact comparison: the operator uses == on doubles.
  SUBCASE("Equality Operators") {
    pf::Position p3{10.0, 20.0};
    pf::Position p4{10.0, 20.001};

    CHECK(p1 == p3);
    CHECK_FALSE(p1 == p4);
    CHECK(p1 != p4);
    CHECK_FALSE(p1 != p3);

    // Difference in a single component at a time.
    CHECK(pf::Position{1.0, 2.0} != pf::Position{1.0, 3.0});
    CHECK(pf::Position{1.0, 2.0} != pf::Position{9.0, 2.0});
  }
}


// TEST 2: Velocity struct operators
// Tested functions: operator+, operator-, operator+=, operator* (both).

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


// TEST 3: Domain validity
// Tested function: space_is_valid(Space).

TEST_CASE("Testing Space Validity") {
  CHECK(pf::space_is_valid(pf::Space{0.0, 100.0, 0.0, 100.0}));
  CHECK(pf::space_is_valid(pf::Space{-50.0, 50.0, 100.0, 200.0}));

  // Coincident or inverted boundaries: degenerate domain.
  CHECK_FALSE(pf::space_is_valid(pf::Space{0.0, 0.0, 0.0, 100.0}));
  CHECK_FALSE(pf::space_is_valid(pf::Space{0.0, 100.0, 50.0, 50.0}));
  CHECK_FALSE(pf::space_is_valid(pf::Space{100.0, 0.0, 0.0, 100.0}));
  CHECK_FALSE(pf::space_is_valid(pf::Space{0.0, 100.0, 100.0, 0.0}));
}

// TEST 4: Toroidal space wrap-around
// Tested function: toroidal_space.

TEST_CASE("Testing Toroidal Space Wrap-Around") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // A boid exiting from one of the 4 borders re-enters from the opposite side.
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

    // Displacement greater than a full domain length.
    CHECK(pf::toroidal_space({250.0, 50.0}, space).x == doctest::Approx(50.0));
    CHECK(pf::toroidal_space({-150.0, 50.0}, space).x == doctest::Approx(50.0));
    CHECK(pf::toroidal_space({50.0, 320.0}, space).y == doctest::Approx(20.0));
    CHECK(pf::toroidal_space({50.0, -150.0}, space).y == doctest::Approx(50.0));
  }

  // Diagonal exit: both components must be adjusted.
  SUBCASE("Corner (both components out of range)") {
    pf::Position wrapped = pf::toroidal_space({103.0, -2.0}, space);

    CHECK(wrapped.x == doctest::Approx(3.0));
    CHECK(wrapped.y == doctest::Approx(98.0));
  }

  // An interior point must remain unchanged.
  SUBCASE("Interior point is left unchanged") {
    pf::Position inside{42.0, 17.0};
    pf::Position result = pf::toroidal_space(inside, space);

    CHECK(result == inside);
  }

  // Shifted/negative domain: wrap uses Lx = x_max - x_min, not x_max.
  SUBCASE("Shifted and negative domain") {
    pf::Space shifted{-50.0, 50.0, 100.0, 200.0};

    pf::Position wrapped_x = pf::toroidal_space({55.0, 150.0}, shifted);
    CHECK(wrapped_x.x == doctest::Approx(-45.0));

    pf::Position wrapped_y = pf::toroidal_space({0.0, 95.0}, shifted);
    CHECK(wrapped_y.y == doctest::Approx(195.0));
  }
}

// TEST 5: Minimum distance in toroidal space
// Tested function: toroidal_difference.

TEST_CASE("Testing Toroidal Difference") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // The shortest path goes across the border: 5 - 95 = -90, which
  // exceeds -Lx/2, so it is wrapped around to +10.
  SUBCASE("Shortest path along X and Y") {
    pf::Position diff_x =
        pf::toroidal_difference({5.0, 50.0}, {95.0, 50.0}, space);
    CHECK(diff_x.x == doctest::Approx(10.0));
    CHECK(diff_x.y == doctest::Approx(0.0));

    pf::Position diff_y =
        pf::toroidal_difference({50.0, 5.0}, {50.0, 95.0}, space);
    CHECK(diff_y.x == doctest::Approx(0.0));
    CHECK(diff_y.y == doctest::Approx(10.0));

    // Symmetric case: reversing the arguments only changes the sign.
    pf::Position diff_rev =
        pf::toroidal_difference({95.0, 50.0}, {5.0, 50.0}, space);
    CHECK(diff_rev.x == doctest::Approx(-10.0));
  }

  // If the direct path is already the shortest, it is not adjusted.
  SUBCASE("Direct path is already the shortest") {
    pf::Position diff =
        pf::toroidal_difference({30.0, 60.0}, {20.0, 40.0}, space);

    CHECK(diff.x == doctest::Approx(10.0));
    CHECK(diff.y == doctest::Approx(20.0));
  }

  // Coincident positions: zero difference.
  SUBCASE("Identical positions") {
    pf::Position p{25.0, 75.0};
    pf::Position diff = pf::toroidal_difference(p, p, space);

    CHECK(diff.x == doctest::Approx(0.0));
    CHECK(diff.y == doctest::Approx(0.0));
  }

  // Exactly at Lx/2 both directions are equivalent: the condition is
  // a strict inequality, so the value remains -50 and is not wrapped to +50.
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

// TEST 6: Toroidal squared distance and speed magnitude
// Tested functions: toroidal_distance_squared, speed_modulus.

TEST_CASE("Testing Toroidal Distance Squared and Speed Modulus") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  SUBCASE("Distance squared, direct and across the border") {
    // 3-4-5 triangle: distance 5, square 25.
    CHECK(pf::toroidal_distance_squared({0.0, 0.0}, {3.0, 4.0}, space) ==
          doctest::Approx(25.0));

    // Across the border: dx = 2, dy = 0 -> 4.
    CHECK(pf::toroidal_distance_squared({99.0, 50.0}, {1.0, 50.0}, space) ==
          doctest::Approx(4.0));

    // Symmetry under argument swapping.
    CHECK(pf::toroidal_distance_squared({10.0, 20.0}, {40.0, 60.0}, space) ==
          doctest::Approx(pf::toroidal_distance_squared({40.0, 60.0},
                                                        {10.0, 20.0}, space)));

    // Distance from a point to itself.
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

// TEST 7: Neighbour detection
// Tested function: neighbours_control.

TEST_CASE("Testing Neighbours Detection and Control") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // Standard case: only the boid within radius d is selected.
  SUBCASE("Neighbours found in standard Euclidean distance") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{0.0, 0.0}, {53.0, 54.0}},
                                   {{0.0, 0.0}, {80.0, 80.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.size() == 1);
    CHECK(neighbours[0] == 1);
  }

  // Two boids on opposite sides of the map detect each other as neighbours
  SUBCASE("Neighbours found across the toroidal border") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {98.0, 50.0}},
                                   {{0.0, 0.0}, {2.0, 50.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.size() == 1);
    CHECK(neighbours[0] == 1);
  }

  // No neighbours beyond visual range.
  SUBCASE("No neighbours are found outside the visual range") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{0.0, 0.0}, {50.0, 50.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.empty());
  }

  // At distance exactly d, the boid is NOT counted as a neighbour.
  SUBCASE("Distance exactly equal to d is excluded") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{0.0, 0.0}, {60.0, 50.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.empty());
  }

  // Multiple neighbours: indices are returned in increasing order and
  // exclude the one out of range.
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

  // Only one boid in the flock: no neighbours possible.
  SUBCASE("Single boid has no neighbours") {
    std::vector<pf::Boid> boids = {{{1.0, 1.0}, {50.0, 50.0}}};

    std::vector<int> neighbours = pf::neighbours_control(0, 10.0, boids, space);

    CHECK(neighbours.empty());
  }
}

// TEST 8: Reynolds Rule 1 - Separation
// Tested function: separation.

TEST_CASE("Testing Rule 1: Separation") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // v1 = -s * sum(neighbour_pos - boid_pos): the boid is pushed away
  // from the neighbour.
  SUBCASE("Basic repulsion") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{0.0, 0.0}, {12.0, 14.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v1 = pf::separation(0.5, 10.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(-1.0));
    CHECK(v1.v_y == doctest::Approx(-2.0));
  }

  // Neighbours beyond d_s do not contribute (distance 8 > d_s = 5).
  SUBCASE("Neighbours beyond d_s are ignored") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{0.0, 0.0}, {18.0, 10.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v1 = pf::separation(0.5, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(0.0));
    CHECK(v1.v_y == doctest::Approx(0.0));
  }

  // Summed contributions from multiple neighbours: one to the right, one above.
  SUBCASE("Contributions of several neighbours are summed") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{0.0, 0.0}, {52.0, 50.0}},
                                   {{0.0, 0.0}, {50.0, 53.0}}};
    std::vector<int> neighbours = {1, 2};

    pf::Velocity v1 = pf::separation(1.0, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(-2.0));
    CHECK(v1.v_y == doctest::Approx(-3.0));
  }

  // Repulsion uses toroidal distance: the neighbour "across the border"
  // pushes toward the interior of the domain.
  SUBCASE("Repulsion across the toroidal border") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {99.0, 50.0}},
                                   {{0.0, 0.0}, {1.0, 50.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v1 = pf::separation(1.0, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(-2.0));
    CHECK(v1.v_y == doctest::Approx(0.0));
  }

  // No neighbours: no change in velocity.
  SUBCASE("No neighbours returns zero") {
    std::vector<pf::Boid> boids = {{{5.0, 5.0}, {10.0, 10.0}}};
    std::vector<int> neighbours = {};

    pf::Velocity v1 = pf::separation(1.0, 5.0, 0, neighbours, boids, space);

    CHECK(v1.v_x == doctest::Approx(0.0));
    CHECK(v1.v_y == doctest::Approx(0.0));
  }
}

// TEST 9: Reynolds Rule 2 - Alignment
// Tested function: alignment.

TEST_CASE("Testing Rule 2: Alignment") {

  // v2 = a * mean(neighbour_v - boid_v) = 0.5 * (10 - 0) = 5.
  SUBCASE("Steering towards the neighbours' mean velocity") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                   {{10.0, 0.0}, {12.0, 10.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v2 = pf::alignment(0.5, 0, neighbours, boids);

    CHECK(v2.v_x == doctest::Approx(5.0));
    CHECK(v2.v_y == doctest::Approx(0.0));
  }

  // Mean over two neighbours: ((4,0) + (0,8)) / 2 = (2,4), scaled by a = 0.5.
  SUBCASE("Mean over several neighbours") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {50.0, 50.0}},
                                   {{4.0, 0.0}, {52.0, 50.0}},
                                   {{0.0, 8.0}, {50.0, 52.0}}};
    std::vector<int> neighbours = {1, 2};

    pf::Velocity v2 = pf::alignment(0.5, 0, neighbours, boids);

    CHECK(v2.v_x == doctest::Approx(1.0));
    CHECK(v2.v_y == doctest::Approx(2.0));
  }

  // If the boid already matches the neighbours' velocity, no steering occurs.
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

// TEST 10: Reynolds Rule 3 - Cohesion
// Tested function: cohesion.

TEST_CASE("Testing Rule 3: Cohesion") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // v3 = c * (neighbours' centre of mass - boid position):
  // ((10,0) + (10,10)) / 2 = (10,5), scaled by c = 0.1.
  SUBCASE("Steering towards the centre of mass") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {0.0, 0.0}},
                                   {{0.0, 0.0}, {10.0, 0.0}},
                                   {{0.0, 0.0}, {10.0, 10.0}}};
    std::vector<int> neighbours = {1, 2};

    pf::Velocity v3 = pf::cohesion(0.1, 0, neighbours, boids, space);

    CHECK(v3.v_x == doctest::Approx(1.0));
    CHECK(v3.v_y == doctest::Approx(0.5));
  }

  // Centre of mass uses toroidal differences: a boid at x = 99 must
  // be attracted to the right (across the border), not to the left.
  SUBCASE("Centre of mass across the toroidal border") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {99.0, 50.0}},
                                   {{0.0, 0.0}, {1.0, 50.0}}};
    std::vector<int> neighbours = {1};

    pf::Velocity v3 = pf::cohesion(0.5, 0, neighbours, boids, space);

    CHECK(v3.v_x == doctest::Approx(1.0));
    CHECK(v3.v_y == doctest::Approx(0.0));
  }

  // Boid already at the neighbours' centre of mass: no correction.
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

// TEST 11: Speed limits
// Tested function: limit_speed (using speed_modulus).

TEST_CASE("Testing Speed Limits") {

  // Above v_max: magnitude is clamped to v_max while preserving direction.
  SUBCASE("Maximum speed limit") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {30.0, 40.0});

    CHECK(pf::speed_modulus(v_lim) == doctest::Approx(10.0));
    CHECK((v_lim.v_x / v_lim.v_y) == doctest::Approx(30.0 / 40.0));
    CHECK(v_lim.v_x == doctest::Approx(6.0));
    CHECK(v_lim.v_y == doctest::Approx(8.0));
  }

  // Below v_min: magnitude is clamped to v_min, direction unchanged.
  SUBCASE("Minimum speed limit") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {1.2, 1.6});

    CHECK(pf::speed_modulus(v_lim) == doctest::Approx(3.0));
    CHECK((v_lim.v_x / v_lim.v_y) == doctest::Approx(1.2 / 1.6));
    CHECK(v_lim.v_x == doctest::Approx(1.8));
    CHECK(v_lim.v_y == doctest::Approx(2.4));
  }

  // Speed already within range: unchanged.
  SUBCASE("Speed already inside the range is unchanged") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {3.0, 4.0});

    CHECK(v_lim.v_x == doctest::Approx(3.0));
    CHECK(v_lim.v_y == doctest::Approx(4.0));
  }

  // Direction (sign of components) is preserved in the third quadrant too.
  SUBCASE("Direction is preserved for negative components") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {-30.0, -40.0});

    CHECK(v_lim.v_x == doctest::Approx(-6.0));
    CHECK(v_lim.v_y == doctest::Approx(-8.0));
  }

  // Zero magnitude: non-normalizable, the function must return the vector
  // unchanged without division by zero.
  SUBCASE("Null modulus does not produce NaN") {
    pf::Velocity v_lim = pf::limit_speed(3.0, 10.0, {0.0, 0.0});

    CHECK(pf::speed_modulus(v_lim) == doctest::Approx(0.0));
    CHECK(v_lim.v_x == doctest::Approx(0.0));
    CHECK(v_lim.v_y == doctest::Approx(0.0));
  }
}

// TEST 12: Random boid generation
// Tested function: generate_boid.

TEST_CASE("Testing Boid Generation") {

  // Generation draws a uniform magnitude in [v_min, v_max] and a uniform
  // angle in [0, 2pi): the constraint applies to magnitude, not components.
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

  // Shifted domain with negative coordinates: distributions must use
  // effective bounds, not [0, x_max].
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

  // v_min == v_max: all boids share the same speed magnitude, different directions.
  SUBCASE("Degenerate speed range (v_min == v_max)") {
    pf::Space space{0.0, 100.0, 0.0, 100.0};

    std::vector<pf::Boid> boids = pf::generate_boid(20, 5.0, 5.0, space);

    for (pf::Boid const& b : boids) {
      CHECK(pf::speed_modulus(b.vel) == doctest::Approx(5.0));
    }
  }
}

// TEST 13: Valid Flock construction and getters
// Tested functions: Flock::Flock, Flock::boids, Flock::parameters,
// Flock::space.

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

  // Constructor must populate the flock with exactly n boids.
  CHECK(flock.boids().size() == static_cast<std::size_t>(par.n_boids));

  // Getters return the input parameters and domain.
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

// TEST 14: Flock class invariants
// Tested functions: Flock::Flock and check_parameters (exceptions).

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

  // Baseline check: valid parameters do not throw.
  CHECK_NOTHROW(pf::Flock(valid_par, space));

  // n <= 0 -> runtime_error (checked before check_parameters).
  SUBCASE("Invalid number of boids") {
    pf::Parameters par_zero = valid_par;
    par_zero.n_boids = 0;
    CHECK_THROWS_AS(pf::Flock(par_zero, space), std::invalid_argument);

    pf::Parameters par_neg = valid_par;
    par_neg.n_boids = -5;
    CHECK_THROWS_AS(pf::Flock(par_neg, space), std::invalid_argument);
  }

  // Degenerate domain.
  SUBCASE("Invalid space") {
    CHECK_THROWS_AS(pf::Flock(valid_par, pf::Space{0.0, 0.0, 0.0, 100.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(pf::Flock(valid_par, pf::Space{0.0, 100.0, 50.0, 20.0}),
                    std::invalid_argument);
  }

  // Factors s, a, c must be strictly positive.
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

  // Radii must be positive.
  SUBCASE("Non-positive radii") {
    pf::Parameters par_d = valid_par;
    par_d.d = -100.0;
    CHECK_THROWS_AS(pf::Flock(par_d, space), std::invalid_argument);

    pf::Parameters par_ds = valid_par;
    par_ds.d_s = 0.0;
    CHECK_THROWS_AS(pf::Flock(par_ds, space), std::invalid_argument);
  }

  // Core model invariant: d_s < d.
  SUBCASE("Separation radius must be smaller than perception radius") {
    pf::Parameters par_eq = valid_par;
    par_eq.d = 20.0;  // d == d_s
    CHECK_THROWS_AS(pf::Flock(par_eq, space), std::invalid_argument);

    pf::Parameters par_lt = valid_par;
    par_lt.d = 10.0;  // d < d_s
    CHECK_THROWS_AS(pf::Flock(par_lt, space), std::invalid_argument);
  }

  // Speed constraints: v_max > 0, v_min >= 0, v_min < v_max.
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

    // v_min = 0 is allowed by the invariants.
    pf::Parameters par_vzero = valid_par;
    par_vzero.v_min = 0.0;
    CHECK_NOTHROW(pf::Flock(par_vzero, space));
  }

  // Time step must be positive.
  SUBCASE("Non-positive dt") {
    pf::Parameters par_dt = valid_par;
    par_dt.dt = -1.0;
    CHECK_THROWS_AS(pf::Flock(par_dt, space), std::invalid_argument);

    pf::Parameters par_dt0 = valid_par;
    par_dt0.dt = 0.0;
    CHECK_THROWS_AS(pf::Flock(par_dt0, space), std::invalid_argument);
  }
}

// TEST 15: Time evolution
// Tested function: Flock::movement.

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

  // After the update, boids remain in the domain and within speed limits,
  // and their count does not change (Iterated multiple times to avoid testing
  // only the first step).
  SUBCASE("Movement preserves flock constraints") {
    pf::Flock flock(par, space);

    for (int step = 0; step != 10; ++step) {
      flock.movement();
    }

    std::vector<pf::Boid> const& boids_after = flock.boids();

    CHECK(boids_after.size() == static_cast<std::size_t>(par.n_boids));

    for (pf::Boid const& b : boids_after) {
      CHECK(b.pos.x >= space.x_min);
      CHECK(b.pos.x <= space.x_max);
      CHECK(b.pos.y >= space.y_min);
      CHECK(b.pos.y <= space.y_max);

      CHECK(pf::speed_modulus(b.vel) <= doctest::Approx(par.v_max));
      CHECK(pf::speed_modulus(b.vel) >= doctest::Approx(par.v_min));
    }
  }

  // Explicit verification of integration:
  // pos_new = toroidal_space(pos_old + dt * v_new).
  SUBCASE("Positions are updated with dt and the new velocity") {
    pf::Flock flock(par, space);

    std::vector<pf::Boid> const before = flock.boids();  // copy

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

  // An isolated boid has no neighbours, so velocity is unchanged.
  SUBCASE("A single boid keeps its velocity") {
    pf::Parameters par_one = par;
    par_one.n_boids = 1;
    pf::Flock flock(par_one, space);

    pf::Velocity const v_before = flock.boids()[0].vel;

    flock.movement();

    pf::Velocity const v_after = flock.boids()[0].vel;

    CHECK(v_after.v_x == doctest::Approx(v_before.v_x));
    CHECK(v_after.v_y == doctest::Approx(v_before.v_y));
  }
}

// TEST 16: Velocity statistics
// Tested functions: mean_velocity, std_dev_velocity.

TEST_CASE("Testing Velocity Statistics") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // Identical magnitudes -> mean equal to magnitude, zero standard deviation.
  SUBCASE("Constant modulus gives zero standard deviation") {
    std::vector<pf::Boid> boids = {{{3.0, 4.0}, {0.0, 0.0}},
                                   {{3.0, 4.0}, {0.0, 0.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_vel = stats.mean_velocity;

    CHECK(m_vel == doctest::Approx(5.0));
    CHECK(stats.std_dev_velocity == doctest::Approx(0.0));
  }
  // Magnitudes 5 and 10 -> mean 7.5; deviations +/- 2.5 -> sqrt(12.5 / 2) = 2.5.
  SUBCASE("Non-zero standard deviation (two boids)") {
    std::vector<pf::Boid> boids = {{{3.0, 4.0}, {0.0, 0.0}},
                                   {{6.0, 8.0}, {10.0, 0.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_vel = stats.mean_velocity;

    CHECK(m_vel == doctest::Approx(7.5));
    CHECK(stats.std_dev_velocity == doctest::Approx(2.5));
  }

  // Magnitudes 3, 5, 7 -> mean 5; deviations -2, 0, +2 -> sqrt(8 / 3) = 1.632993.
  SUBCASE("Non-zero standard deviation (three boids)") {
    std::vector<pf::Boid> boids = {{{3.0, 0.0}, {0.0, 0.0}},
                                   {{0.0, 5.0}, {10.0, 0.0}},
                                   {{0.0, -7.0}, {20.0, 0.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_vel = stats.mean_velocity;

    CHECK(m_vel == doctest::Approx(5.0));
    CHECK(stats.std_dev_velocity == doctest::Approx(1.632993).epsilon(0.001));
  }

  // Magnitude is always non-negative: opposite velocities yield the same mean.
  SUBCASE("Opposite velocities have the same modulus") {
    std::vector<pf::Boid> boids = {{{3.0, 4.0}, {0.0, 0.0}},
                                   {{-3.0, -4.0}, {10.0, 0.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_vel = stats.mean_velocity;

    CHECK(m_vel == doctest::Approx(5.0));
    CHECK(stats.std_dev_velocity == doctest::Approx(0.0));
  }
}

// TEST 17: Distance statistics
// Tested functions: mean_distance, std_dev_distance.

TEST_CASE("Testing Distance Statistics") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  // Two boids, a single pair: mean = distance, zero standard deviation.
  SUBCASE("Two boids: mean equals the distance") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {0.0, 0.0}},
                                   {{0.0, 0.0}, {3.0, 4.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_dist = stats.mean_distance;

    CHECK(m_dist == doctest::Approx(5.0).epsilon(0.001));
    CHECK(stats.std_dev_distance == doctest::Approx(0.0));
  }

  // Distances are toroidal: 1 and 99 are 2 units apart, not 98.
  SUBCASE("Distances are computed on the torus") {
    std::vector<pf::Boid> boids = {{{0.0, 0.0}, {1.0, 50.0}},
                                   {{0.0, 0.0}, {99.0, 50.0}}};

    pf::Statistics stats = pf::statistics(boids, space);
    double const m_dist = stats.mean_distance;

    CHECK(m_dist == doctest::Approx(2.0));
  }

  // Three boids at the vertices of a 6-8-10 right triangle: pairwise
  // distances are 6, 8, 10 -> mean 8; deviations -2, 0, +2 -> sqrt(8 / 3).
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

// TEST 18: Statistics edge cases

// Single boid: no pairs, preventing division by zero.
TEST_CASE("Testing Statistics Edge Cases (1 boid)") {
  pf::Space space{0.0, 100.0, 0.0, 100.0};

  std::vector<pf::Boid> single_boid = {{{3.0, 4.0}, {10.0, 10.0}}};

  pf::Statistics stats = pf::statistics(single_boid, space);
  double const m_dist = stats.mean_distance;

  CHECK(m_dist == doctest::Approx(0.0));
  CHECK(stats.std_dev_distance == doctest::Approx(0.0));

  // With a single boid, the mean velocity equals its own magnitude.
  double const m_vel = stats.mean_velocity;

  CHECK(m_vel == doctest::Approx(5.0));
  CHECK(stats.std_dev_velocity == doctest::Approx(0.0));
}
// ===========================================================================
// TEST 19: Validazione dei parametri del predatore
// Funzione testata: check_predator_parameters.
// Si parte da un set valido e si rompe un vincolo alla volta, come nel test
// gemello su check_parameters.
// ===========================================================================

TEST_CASE("Testing check_predator_parameters") {
  pf::Space const space{0.0, 1200.0, 0.0, 1000.0};

  pf::Predator_parameters const valid_par_p{
      1,      // n_predators
      0.3,    // s_p
      0.007,  // c_p
      200.0,  // d_chase
      70.0,   // d_escape
      22.0,   // v_min_p
      45.0    // v_max_p
  };

  SUBCASE("Valid parameters") {
    CHECK_NOTHROW(pf::check_predator_parameters(valid_par_p, space));
  }

  SUBCASE("Non-positive number of predators") {
    pf::Predator_parameters par_p = valid_par_p;
    par_p.n_predators = 0;
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);

    par_p.n_predators = -3;
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);
  }

  SUBCASE("Non-positive s_p or c_p") {
    pf::Predator_parameters par_p = valid_par_p;
    par_p.s_p = 0.0;
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);

    par_p = valid_par_p;
    par_p.c_p = -0.001;
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);
  }

  SUBCASE("Non-positive d_chase or d_escape") {
    pf::Predator_parameters par_p = valid_par_p;
    par_p.d_chase = 0.0;
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);

    par_p = valid_par_p;
    par_p.d_escape = -10.0;
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);
  }

  SUBCASE("Invalid predator speeds") {
    pf::Predator_parameters par_p = valid_par_p;
    par_p.v_min_p = 0.0;
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);

    par_p = valid_par_p;
    par_p.v_max_p = -1.0;
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);

    par_p = valid_par_p;
    par_p.v_min_p = 50.0;  // maggiore di v_max_p
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);
  }

  // d_chase piu' grande di meta' del lato minore renderebbe ambiguo il
  // centro di massa sul toro: due prede possono trovarsi ai due lati
  // opposti e la direzione di inseguimento non sarebbe piu' definita.
  SUBCASE("d_chase larger than half the domain") {
    pf::Predator_parameters par_p = valid_par_p;
    par_p.d_chase = 501.0;  // meta' del lato minore e' 500
    CHECK_THROWS_AS(pf::check_predator_parameters(par_p, space),
                    std::invalid_argument);

    par_p.d_chase = 500.0;  // esattamente al limite: accettato
    CHECK_NOTHROW(pf::check_predator_parameters(par_p, space));
  }

  // Il dominio entra nel controllo: gli stessi parametri possono essere
  // validi in uno spazio grande e invalidi in uno piccolo.
  SUBCASE("The same parameters depend on the space") {
    pf::Space const small_space{0.0, 300.0, 0.0, 300.0};
    CHECK_THROWS_AS(pf::check_predator_parameters(valid_par_p, small_space),
                    std::invalid_argument);
  }
}

// ===========================================================================
// TEST 20: Individuazione delle prede
// Funzione testata: preys_control.
// Analogo di neighbours_control, ma il centro e' il predatore e non un boid,
// quindi nessun indice viene escluso a priori.
// ===========================================================================

TEST_CASE("Testing preys_control") {
  pf::Space const space{0.0, 100.0, 0.0, 100.0};
  pf::Boid const predator{{0.0, 0.0}, {50.0, 50.0}};

  SUBCASE("Some boids inside, some outside") {
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {55.0, 50.0}},   // dentro
                                         {{0.0, 0.0}, {50.0, 70.0}},   // fuori
                                         {{0.0, 0.0}, {40.0, 45.0}}};  // dentro
    std::vector<int> const preys =
        pf::preys_control(15.0, predator, boids, space);

    REQUIRE(preys.size() == 2);
    CHECK(preys[0] == 0);
    CHECK(preys[1] == 2);
  }

  SUBCASE("No boid in range") {
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {10.0, 10.0}},
                                         {{0.0, 0.0}, {90.0, 90.0}}};
    std::vector<int> const preys =
        pf::preys_control(5.0, predator, boids, space);

    CHECK(preys.empty());
  }

  SUBCASE("No boid at all") {
    std::vector<pf::Boid> const boids{};
    std::vector<int> const preys =
        pf::preys_control(50.0, predator, boids, space);

    CHECK(preys.empty());
  }

  // A differenza di neighbours_control, il predatore non sta dentro boids:
  // nessun indice va escluso, nemmeno se coincide con la sua posizione.
  SUBCASE("A boid at the same position as the predator is a prey") {
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {50.0, 50.0}}};
    std::vector<int> const preys =
        pf::preys_control(15.0, predator, boids, space);

    REQUIRE(preys.size() == 1);
    CHECK(preys[0] == 0);
  }

  SUBCASE("Distance is toroidal") {
    // Predatore in x = 95, preda in x = 5: sul toro distano 10, non 90.
    pf::Boid const border_predator{{0.0, 0.0}, {95.0, 50.0}};
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {5.0, 50.0}}};
    std::vector<int> const preys =
        pf::preys_control(20.0, border_predator, boids, space);

    REQUIRE(preys.size() == 1);
    CHECK(preys[0] == 0);
  }

  SUBCASE("A boid exactly at d_chase is excluded") {
    // Il confronto e' stretto: a distanza esattamente d_chase sta fuori.
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {60.0, 50.0}}};
    std::vector<int> const preys =
        pf::preys_control(10.0, predator, boids, space);

    CHECK(preys.empty());
  }
}

// ===========================================================================
// TEST 21: Inseguimento del predatore
// Funzione testata: chase.
// Restituisce il contributo di velocita' diretto dal predatore verso il
// centro di massa delle prede, con modulo c_p per la distanza da quel centro.
// La differenza toroidale e' presa rispetto al predatore, quindi il risultato
// resta corretto anche quando le prede stanno a cavallo del bordo.
// ===========================================================================

TEST_CASE("Testing chase") {
  pf::Space const space{0.0, 100.0, 0.0, 100.0};
  pf::Boid const predator{{0.0, 0.0}, {50.0, 50.0}};

  SUBCASE("No preys: no contribution") {
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {10.0, 10.0}}};
    pf::Velocity const v = pf::chase(0.5, predator, {}, boids, space);

    CHECK(v.v_x == doctest::Approx(0.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Direction and magnitude") {
    // Centro di massa delle prede in (70, 60): 20 a destra e 10 sotto.
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {70.0, 50.0}},
                                         {{0.0, 0.0}, {70.0, 70.0}}};
    pf::Velocity const v = pf::chase(0.5, predator, {0, 1}, boids, space);

    CHECK(v.v_x == doctest::Approx(10.0));
    CHECK(v.v_y == doctest::Approx(5.0));
  }

  SUBCASE("Predator already on the centre of mass") {
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {40.0, 50.0}},
                                         {{0.0, 0.0}, {60.0, 50.0}}};
    pf::Velocity const v = pf::chase(0.5, predator, {0, 1}, boids, space);

    CHECK(v.v_x == doctest::Approx(0.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Only the indexed preys count") {
    // Il boid 1 non compare fra le prede: non deve pesare sul centro di massa.
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {60.0, 50.0}},
                                         {{0.0, 0.0}, {10.0, 10.0}},
                                         {{0.0, 0.0}, {80.0, 50.0}}};
    pf::Velocity const v = pf::chase(1.0, predator, {0, 2}, boids, space);

    CHECK(v.v_x == doctest::Approx(20.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Chase takes the short way around the torus") {
    // Predatore in x = 95, preda in x = 5: deve andare a destra (+10) e
    // uscire dal bordo, non tornare indietro di 90.
    pf::Boid const border_predator{{0.0, 0.0}, {95.0, 50.0}};
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {5.0, 50.0}}};
    pf::Velocity const v = pf::chase(1.0, border_predator, {0}, boids, space);

    CHECK(v.v_x == doctest::Approx(10.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Preys across the border average correctly") {
    // Prede in x = 95 e x = 5 con predatore in x = 0: il centro di massa
    // e' in x = 0, quindi il contributo e' nullo. La media aritmetica
    // ingenua darebbe 50 e una spinta sbagliata.
    pf::Boid const border_predator{{0.0, 0.0}, {0.0, 50.0}};
    std::vector<pf::Boid> const boids = {{{0.0, 0.0}, {95.0, 50.0}},
                                         {{0.0, 0.0}, {5.0, 50.0}}};
    pf::Velocity const v =
        pf::chase(1.0, border_predator, {0, 1}, boids, space);

    CHECK(v.v_x == doctest::Approx(0.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }
}

// ===========================================================================
// TEST 22: Fuga dai predatori
// Funzione testata: escape.
// Stessa forma della separazione, ma rispetto ai predatori e con raggio
// proprio d_escape. I contributi dei singoli predatori si sommano.
// Il controllo sul verso e' quello che protegge dallo scambio accidentale
// degli argomenti boid/predator e dall'inversione del confronto sul raggio.
// ===========================================================================

TEST_CASE("Testing escape") {
  pf::Space const space{0.0, 400.0, 0.0, 400.0};
  pf::Boid const boid{{0.0, 0.0}, {200.0, 200.0}};

  SUBCASE("No predators: no contribution") {
    std::vector<pf::Boid> const predators{};
    pf::Velocity const v = pf::escape(1.0, 60.0, boid, predators, space);

    CHECK(v.v_x == doctest::Approx(0.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Predator on the right: the boid flees to the left") {
    std::vector<pf::Boid> const predators{{{0.0, 0.0}, {220.0, 200.0}}};
    pf::Velocity const v = pf::escape(1.0, 60.0, boid, predators, space);

    CHECK(v.v_x == doctest::Approx(-20.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Predator out of range: no contribution") {
    std::vector<pf::Boid> const predators{{{0.0, 0.0}, {380.0, 200.0}}};
    pf::Velocity const v = pf::escape(1.0, 60.0, boid, predators, space);

    CHECK(v.v_x == doctest::Approx(0.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Predator exactly at d_escape: no contribution") {
    // Il confronto e' stretto: sul bordo del raggio il boid non reagisce.
    std::vector<pf::Boid> const predators{{{0.0, 0.0}, {260.0, 200.0}}};
    pf::Velocity const v = pf::escape(1.0, 60.0, boid, predators, space);

    CHECK(v.v_x == doctest::Approx(0.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("s_p scales the contribution") {
    std::vector<pf::Boid> const predators{{{0.0, 0.0}, {210.0, 190.0}}};
    pf::Velocity const v = pf::escape(0.2, 60.0, boid, predators, space);

    CHECK(v.v_x == doctest::Approx(-2.0));
    CHECK(v.v_y == doctest::Approx(2.0));
  }

  SUBCASE("Predators on the same side add up") {
    std::vector<pf::Boid> const predators{{{0.0, 0.0}, {220.0, 200.0}},
                                          {{0.0, 0.0}, {230.0, 200.0}}};
    pf::Velocity const v = pf::escape(1.0, 60.0, boid, predators, space);

    // -20 dal primo, -30 dal secondo
    CHECK(v.v_x == doctest::Approx(-50.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Predators from different directions turn the escape") {
    // Uno a destra e uno sotto: la risultante punta in diagonale, lontano
    // da entrambi.
    std::vector<pf::Boid> const predators{{{0.0, 0.0}, {220.0, 200.0}},
                                          {{0.0, 0.0}, {200.0, 240.0}}};
    pf::Velocity const v = pf::escape(1.0, 60.0, boid, predators, space);

    CHECK(v.v_x == doctest::Approx(-20.0));
    CHECK(v.v_y == doctest::Approx(-40.0));
  }

  // Comportamento voluto del modello a somma vettoriale, lo stesso che ha
  // separation: predatori disposti simmetricamente si annullano. Se un
  // giorno si passa a "reagisci solo al piu' vicino", questo test cambia.
  SUBCASE("Symmetric predators cancel out") {
    std::vector<pf::Boid> const predators{{{0.0, 0.0}, {220.0, 200.0}},
                                          {{0.0, 0.0}, {180.0, 200.0}}};
    pf::Velocity const v = pf::escape(1.0, 60.0, boid, predators, space);

    CHECK(v.v_x == doctest::Approx(0.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Only predators within d_escape contribute") {
    std::vector<pf::Boid> const predators{{{0.0, 0.0}, {220.0, 200.0}},
                                          {{0.0, 0.0}, {380.0, 200.0}}};
    pf::Velocity const v = pf::escape(1.0, 60.0, boid, predators, space);

    CHECK(v.v_x == doctest::Approx(-20.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }

  SUBCASE("Escape is toroidal") {
    // Boid in x = 10, predatore in x = 390: sul toro distano 20 e il
    // predatore e' a sinistra, quindi il boid deve fuggire a destra.
    pf::Boid const border_boid{{0.0, 0.0}, {10.0, 200.0}};
    std::vector<pf::Boid> const predators{{{0.0, 0.0}, {390.0, 200.0}}};
    pf::Velocity const v = pf::escape(1.0, 60.0, border_boid, predators, space);

    CHECK(v.v_x == doctest::Approx(20.0));
    CHECK(v.v_y == doctest::Approx(0.0));
  }
}

// ===========================================================================
// TEST 23: Il predatore dentro Flock
// Funzioni testate: i due costruttori, predators(), has_predator(),
// predator_parameters(), movement().
// E' la presenza dei Predator_parameters a decidere se i predatori esistono:
// il costruttore a due argomenti non ne genera e non ne convalida i valori.
// ===========================================================================

TEST_CASE("Testing the optional predators inside Flock") {
  pf::Space const space{0.0, 1200.0, 0.0, 1000.0};

  pf::Parameters const par{
      100,    // n_boids
      0.2,    // s
      0.1,    // a
      0.01,   // c
      100.0,  // d
      40.0,   // d_s
      20.0,   // v_min
      42.0,   // v_max
      0.2     // dt
  };

  pf::Predator_parameters const par_p{
      1,      // n_predators
      0.3,    // s_p
      0.007,  // c_p
      200.0,  // d_chase
      70.0,   // d_escape
      22.0,   // v_min_p
      45.0    // v_max_p
  };

  SUBCASE("Two-argument constructor: no predators") {
    pf::Flock flock(par, space);

    CHECK(flock.has_predator() == false);
    CHECK(flock.predators().empty());
    CHECK(flock.boids().size() == static_cast<std::size_t>(par.n_boids));
  }

  SUBCASE("Three-argument constructor: predators present") {
    pf::Flock flock(par, space, par_p);

    CHECK(flock.has_predator() == true);
    CHECK(flock.predators().size() ==
          static_cast<std::size_t>(par_p.n_predators));
    CHECK(flock.predator_parameters().d_chase ==
          doctest::Approx(par_p.d_chase));

    for (pf::Boid const& predator : flock.predators()) {
      CHECK(predator.pos.x >= space.x_min);
      CHECK(predator.pos.x <= space.x_max);
      CHECK(predator.pos.y >= space.y_min);
      CHECK(predator.pos.y <= space.y_max);

      double const speed = pf::speed_modulus(predator.vel);
      CHECK(speed >= par_p.v_min_p);
      CHECK(speed <= par_p.v_max_p);
    }
  }

  SUBCASE("n_predators governs how many are created") {
    pf::Predator_parameters par_p_many = par_p;
    par_p_many.n_predators = 5;

    pf::Flock flock(par, space, par_p_many);

    CHECK(flock.predators().size() == 5);
  }

  // Senza predatori i loro parametri non vengono nemmeno guardati: valori
  // che col costruttore a tre argomenti farebbero lanciare un'eccezione
  // qui devono passare senza problemi.
  SUBCASE("Predator parameters are not validated when there are none") {
    pf::Predator_parameters const bad_par_p{-1,   -1.0, -1.0, -1.0,
                                            -1.0, 50.0, 10.0};

    CHECK_THROWS_AS(pf::Flock(par, space, bad_par_p), std::invalid_argument);
    CHECK_NOTHROW(pf::Flock(par, space));
  }

  // Il controllo decisivo sul ramo senza predatori: escape non viene mai
  // applicata. Un boid isolato, con un raggio di percezione piu' grande
  // dell'intero dominio, deve comunque mantenere la sua velocita'.
  SUBCASE("Without predators escape is never applied") {
    pf::Parameters par_one = par;
    par_one.n_boids = 1;

    pf::Flock flock(par_one, space);

    pf::Velocity const v_before = flock.boids()[0].vel;
    flock.movement();
    pf::Velocity const v_after = flock.boids()[0].vel;

    CHECK(v_after.v_x == doctest::Approx(v_before.v_x));
    CHECK(v_after.v_y == doctest::Approx(v_before.v_y));
  }

  SUBCASE("With a predator in range the isolated boid does react") {
    pf::Parameters par_one = par;
    par_one.n_boids = 1;

    pf::Predator_parameters par_p_wide = par_p;
    par_p_wide.d_escape = 2000.0;  // copre tutto il dominio

    pf::Flock flock(par_one, space, par_p_wide);

    pf::Velocity const v_before = flock.boids()[0].vel;
    flock.movement();
    pf::Velocity const v_after = flock.boids()[0].vel;

    bool const changed = v_after.v_x != doctest::Approx(v_before.v_x) ||
                         v_after.v_y != doctest::Approx(v_before.v_y);
    CHECK(changed);
  }

  SUBCASE("The flock evolves normally with no predators") {
    pf::Flock flock(par, space);

    for (int step = 0; step != 50; ++step) {
      flock.movement();
      CHECK(flock.has_predator() == false);
    }

    for (pf::Boid const& b : flock.boids()) {
      CHECK(b.pos.x >= space.x_min);
      CHECK(b.pos.x <= space.x_max);
      CHECK(b.pos.y >= space.y_min);
      CHECK(b.pos.y <= space.y_max);
      CHECK(pf::speed_modulus(b.vel) <= doctest::Approx(par.v_max));
    }
  }

  SUBCASE("Several predators all stay regular while moving") {
    pf::Predator_parameters par_p_many = par_p;
    par_p_many.n_predators = 4;

    pf::Flock flock(par, space, par_p_many);

    for (int step = 0; step != 50; ++step) {
      flock.movement();
    }

    CHECK(flock.predators().size() == 4);

    for (pf::Boid const& predator : flock.predators()) {
      CHECK(predator.pos.x >= space.x_min);
      CHECK(predator.pos.x <= space.x_max);
      CHECK(predator.pos.y >= space.y_min);
      CHECK(predator.pos.y <= space.y_max);
      CHECK(pf::speed_modulus(predator.vel) <=
            doctest::Approx(par_p_many.v_max_p));
    }
  }
}

// ===========================================================================
// TEST 24: Lettura dei parametri da file
// Funzione testata: file_input.
// I file di prova vengono scritti su disco dal test stesso e rimossi alla
// fine, cosi' la suite non dipende da file esterni.
// ===========================================================================

TEST_CASE("Testing file_input") {
  // Blocco dei soli parametri dello stormo e del dominio.
  std::string const boids_block =
      "boids: 150\n"
      "separation: 0.2\n"
      "alignment: 0.1\n"
      "cohesion: 0.01\n"
      "neighbours_distance: 100.0\n"
      "separation_distance: 40.0\n"
      "v_min: 20.0\n"
      "v_max: 42.0\n"
      "dt: 0.2\n"
      "x_min: 0.0\n"
      "x_max: 1200.0\n"
      "y_min: 0.0\n"
      "y_max: 1000.0\n";

  std::string const predator_block =
      "predators: 3\n"
      "separation_predator: 0.3\n"
      "cohesion_predator: 0.007\n"
      "d_chase: 200.0\n"
      "d_escape: 70.0\n"
      "v_min_p: 22.0\n"
      "v_max_p: 45.0\n";

  auto write_file = [](std::string const& name, std::string const& content) {
    std::ofstream out(name);
    out << content;
  };

  pf::Parameters par{};
  pf::Predator_parameters par_p{};
  pf::Space space{};

  SUBCASE("Complete file, predators requested") {
    std::string name = "test_complete.txt";
    write_file(name, boids_block + predator_block);

    CHECK_NOTHROW(pf::file_input(name, par, space, par_p, true));

    CHECK(par.n_boids == 150);
    CHECK(par.s == doctest::Approx(0.2));
    CHECK(par.a == doctest::Approx(0.1));
    CHECK(par.c == doctest::Approx(0.01));
    CHECK(par.d == doctest::Approx(100.0));
    CHECK(par.d_s == doctest::Approx(40.0));
    CHECK(par.v_min == doctest::Approx(20.0));
    CHECK(par.v_max == doctest::Approx(42.0));
    CHECK(par.dt == doctest::Approx(0.2));

    CHECK(space.x_min == doctest::Approx(0.0));
    CHECK(space.x_max == doctest::Approx(1200.0));
    CHECK(space.y_min == doctest::Approx(0.0));
    CHECK(space.y_max == doctest::Approx(1000.0));

    CHECK(par_p.n_predators == 3);
    CHECK(par_p.s_p == doctest::Approx(0.3));
    CHECK(par_p.c_p == doctest::Approx(0.007));
    CHECK(par_p.d_chase == doctest::Approx(200.0));
    CHECK(par_p.d_escape == doctest::Approx(70.0));
    CHECK(par_p.v_min_p == doctest::Approx(22.0));
    CHECK(par_p.v_max_p == doctest::Approx(45.0));

    std::remove(name.c_str());
  }

  // Le righe del predatore restano nel file ma non vanno lette.
  SUBCASE("Complete file, predators not requested") {
    std::string name = "test_complete_no_pred.txt";
    write_file(name, boids_block + predator_block);

    CHECK_NOTHROW(pf::file_input(name, par, space, par_p, false));

    CHECK(par.n_boids == 150);
    CHECK(par.v_max == doctest::Approx(42.0));
    CHECK(space.y_max == doctest::Approx(1000.0));

    // par_p non e' stato toccato
    CHECK(par_p.n_predators == 0);
    CHECK(par_p.d_chase == doctest::Approx(0.0));

    std::remove(name.c_str());
  }

  SUBCASE("File without predator lines, predators not requested") {
    std::string name = "test_short.txt";
    write_file(name, boids_block);

    CHECK_NOTHROW(pf::file_input(name, par, space, par_p, false));
    CHECK(par.n_boids == 150);

    std::remove(name.c_str());
  }

  SUBCASE("File without predator lines, predators requested") {
    std::string name = "test_short_pred.txt";
    write_file(name, boids_block);

    CHECK_THROWS_AS(pf::file_input(name, par, space, par_p, true),
                    std::runtime_error);

    std::remove(name.c_str());
  }

  SUBCASE("Missing file") {
    std::string name = "questo_file_non_esiste_12345.txt";
    CHECK_THROWS_AS(pf::file_input(name, par, space, par_p, false),
                    std::runtime_error);
  }

  SUBCASE("Truncated file") {
    std::string name = "test_truncated.txt";
    write_file(name,
               "boids: 150\n"
               "separation: 0.2\n"
               "alignment: 0.1\n");

    CHECK_THROWS_AS(pf::file_input(name, par, space, par_p, false),
                    std::runtime_error);

    std::remove(name.c_str());
  }

  // Una lettera dove serve un numero manda in fallimento lo stream.
  SUBCASE("Wrong format") {
    std::string name = "test_wrong.txt";
    write_file(name,
               "boids: molti\n"
               "separation: 0.2\n");

    CHECK_THROWS_AS(pf::file_input(name, par, space, par_p, false),
                    std::runtime_error);

    std::remove(name.c_str());
  }

  SUBCASE("Empty file") {
    std::string name = "test_empty.txt";
    write_file(name, "");

    CHECK_THROWS_AS(pf::file_input(name, par, space, par_p, false),
                    std::runtime_error);

    std::remove(name.c_str());
  }
}