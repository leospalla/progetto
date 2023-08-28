#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing the operators") {
  vc::Vector v1{2., 2.};
  vc::Vector v2{1., 3.};
  SUBCASE("+ operator") {
    vc::Vector v3 = v1 + v2;
    vc::Vector v4 = v1 + v2 + v3;
    CHECK(v3.getX() == doctest::Approx(3.));
    CHECK(v3.getY() == doctest::Approx(5.));
    CHECK(v4.getX() == doctest::Approx(6.));
    CHECK(v4.getY() == doctest::Approx(10.));
  }
  SUBCASE("- operator") {
    vc::Vector v3 = v1 - v2;
    vc::Vector v4{1., -1.};
    vc::Vector v5 = v4 - v3;
    CHECK(v3.getX() == doctest::Approx(1.));
    CHECK(v3.getY() == doctest::Approx(-1.));
    CHECK(v5.getX() == doctest::Approx(0.));
    CHECK(v5.getY() == doctest::Approx(0.));
  }
  SUBCASE("* operator") {
    vc::Vector v3 = v1 * 3;
    vc::Vector v4 = v2 * -1.5;
    CHECK(v3.getX() == doctest::Approx(6.));
    CHECK(v3.getY() == doctest::Approx(6.));
    CHECK(v4.getX() == doctest::Approx(-1.5));
    CHECK(v4.getY() == doctest::Approx(-4.5));
  }
  SUBCASE("/ operator") {
    vc::Vector v3 = v1 / 2;
    vc::Vector v4 = v2 / -2;
    CHECK(v3.getX() == doctest::Approx(1.));
    CHECK(v3.getY() == doctest::Approx(1.));
    CHECK(v4.getX() == doctest::Approx(-0.5));
    CHECK(v4.getY() == doctest::Approx(-1.5));
  }
  SUBCASE("+= operator") {
    v2 += v1;
    CHECK(v2.getX() == doctest::Approx(3.));
    CHECK(v2.getY() == doctest::Approx(5.));
    vc::Vector v3{};
    v3 += v2;
    CHECK(v3.getX() == doctest::Approx(3.));
    CHECK(v3.getY() == doctest::Approx(5.));
  }
  SUBCASE("== operator") {
    vc::Vector v3{v1};
    CHECK(v3 == v1);
    CHECK(v2 == vc::Vector(1., 3.));
  }
}
TEST_CASE("Testing the vector functions") {
  vc::Vector v1{2., 2.};
  vc::Vector v2{-1., 3.};
  SUBCASE("Null constructor") {
    vc::Vector v3{};
    CHECK(v3.getX() == doctest::Approx(0.));
    CHECK(v3.getY() == doctest::Approx(0.));
  }
  SUBCASE("Copy constructor") {
    vc::Vector v3{v2};
    CHECK(v3.getX() == doctest::Approx(-1.));
    CHECK(v3.getY() == doctest::Approx(3.));
  }
  SUBCASE("Magnitude function") {
    CHECK(v1.Magnitude() == doctest::Approx(2.828427125));
    CHECK(v2.Magnitude() == doctest::Approx(3.16227766));
  }
  SUBCASE("Normalize function") {
    v1.normalize();
    v2.normalize();
    vc::Vector v3{0., 0.};
    v3.normalize();
    CHECK(v1.getX() == doctest::Approx(0.7071067811));
    CHECK(v1.getY() == doctest::Approx(0.7071067811));
    CHECK(v2.getX() == doctest::Approx(-0.316227766));
    CHECK(v2.getY() == doctest::Approx(0.9486832981));
    CHECK(v1.Magnitude() == doctest::Approx(1.));
    CHECK(v2.Magnitude() == doctest::Approx(1.));
    CHECK(v3.Magnitude() == doctest::Approx(0.));
    CHECK(v3.getX() == doctest::Approx(0.));
    CHECK(v3.getY() == doctest::Approx(0.));
  }
  SUBCASE("Set function") {
    v1.set(3., -3);
    CHECK(v1.getX() == doctest::Approx(3.));
    CHECK(v1.getY() == doctest::Approx(-3.));
  }
  SUBCASE("Distance function") {
    vc::Vector v3 = v1 - v2;
    vc::Vector v4 = v2 - v1;
    vc::Vector v5{0., 2.};
    vc::Vector v6{0., 6.};
    CHECK(v1.distance(v2) == doctest::Approx(3.16227766));
    CHECK(v2.distance(v1) == doctest::Approx(3.16227766));
    CHECK(v3.Magnitude() == doctest::Approx(3.16227766));
    CHECK(v4.Magnitude() == doctest::Approx(3.16227766));
    CHECK(v5.distance(v6) == doctest::Approx(4.));
  }
  SUBCASE("Limit function") {
    vc::Vector v3 = v1 + v2;
    v3.limit(2.);
    CHECK(v3.Magnitude() == doctest::Approx(2.));
    v3.limit(1.);
    CHECK(v3.Magnitude() == doctest::Approx(1.));
  }
}