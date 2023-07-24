#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "vector.hpp"

#include "doctest.h"

TEST_CASE("Testing the operators") {
  Vector v1(2., 2.);
  Vector v2(1., 3.);
  SUBCASE("+ operator") {
    Vector v3 = v1 + v2;
    CHECK(v3.x == doctest::Approx(3.));
    CHECK(v3.y == doctest::Approx(5.));
  }
  SUBCASE("- operator") {
    Vector v3 = v1 - v2;
    CHECK(v3.x == doctest::Approx(1.));
    CHECK(v3.y == doctest::Approx(-1.));
  }
  SUBCASE("* operator") {
    Vector v3 = v1 * 3;
    Vector v4 = v2 * -1.5;
    CHECK(v3.x == doctest::Approx(6.));
    CHECK(v3.y == doctest::Approx(6.));
    CHECK(v4.x == doctest::Approx(-1.5));
    CHECK(v4.y == doctest::Approx(-4.5));
  }
  SUBCASE("/ operator") {
    Vector v3 = v1 / 2;
    Vector v4 = v2 / -2;
    CHECK(v3.x == doctest::Approx(1.));
    CHECK(v3.y == doctest::Approx(1.));
    CHECK(v4.x == doctest::Approx(-0.5));
    CHECK(v4.y == doctest::Approx(-1.5));
  }
}

TEST_CASE("Testing the vector functions") {
  Vector v1(2., 2.);
  Vector v2(-1., 3.);
  SUBCASE("Magnitude function") {
    CHECK(v1.Magnitude() == doctest::Approx(2.828427125));
    CHECK(v2.Magnitude() == doctest::Approx(3.16227766));
  }
  SUBCASE("Normalize function") {
    Vector v3 = v1.Normalize();
    Vector v4 = v2.Normalize();
    CHECK(v3.x == doctest::Approx(0.7071067811));
    CHECK(v3.y == doctest::Approx(0.7071067811));
    CHECK(v4.x == doctest::Approx(-0.316227766));
    CHECK(v4.y == doctest::Approx(0.9486832981));
    CHECK(v3.Magnitude() == doctest::Approx(1.));
    CHECK(v4.Magnitude() == doctest::Approx(1.));
  }
  SUBCASE("Dot product function") {
  CHECK(v1.dotProduct(v2) == doctest::Approx(4.));
  CHECK(v2.dotProduct(v1) == doctest::Approx(v1.dotProduct(v2))); //it should be commutative
  }
}
