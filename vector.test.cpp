#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "vector.hpp"

#include "doctest.h"

TEST_CASE("Testing the operators") {
  Vector v1(2., 2.);
  Vector v2(1., 3.);
  SUBCASE("+ operator") {
    Vector v3 = v1 + v2;
    CHECK(v3.x == doctest::Approx(3.));
  }
}

