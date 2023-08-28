#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "flock.hpp"
TEST_CASE("testing the flock class") {
  fk::Flock flock;
  SUBCASE("testing add boid and remove functions") {
    bd::Boid b1;
    bd::Boid b2;
    flock.addBoid(b1);
    flock.addBoid(b2);
    const std::vector<bd::Boid> &flk1 = flock.getBoids();
    CHECK(flk1.size() == 2.0);
    CHECK(flk1[0] == b1);
    CHECK(flk1[1] == b2);
    flock.removeBoid(b1);
    const std::vector<bd::Boid> &flk2 = flock.getBoids();
    CHECK(flk2.size() == 1.0);
    CHECK(flk2[0] == b2);
  }
  SUBCASE("testing updatevelocity with one boid") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    flock.addBoid(b1);
    flock.updateVelocity();
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].getVelocity().getX() == doctest::Approx(0.0));
    CHECK(flk[0].getVelocity().getY() == doctest::Approx(1.0));
  }
  SUBCASE("testing updatevelocity with distant boids") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1., 1.});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.updateVelocity();
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].getVelocity().getX() == doctest::Approx(0.0));
    CHECK(flk[0].getVelocity().getY() == doctest::Approx(1.0));
    CHECK(flk[1].getVelocity().getX() == doctest::Approx(1.0));
    CHECK(flk[1].getVelocity().getY() == doctest::Approx(1.0));
  }
  SUBCASE("testing updatevelocity with two distant boids and one close boid") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1., 1.});
    bd::Boid b3(vc::Vector{0., 0.}, vc::Vector{1., 1.});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.updateVelocity();
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].getVelocity().getX() == doctest::Approx(0.9));
    CHECK(flk[0].getVelocity().getY() == doctest::Approx(1.0));
    CHECK(flk[1].getVelocity().getX() == doctest::Approx(1.0));
    CHECK(flk[1].getVelocity().getY() == doctest::Approx(1.0));
    CHECK(flk[2].getVelocity().getX() == doctest::Approx(0.1));
    CHECK(flk[2].getVelocity().getY() == doctest::Approx(1.0));
  }
  SUBCASE("testing updatevelocity with 3 boids") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{0., 0.}, vc::Vector{1., 1.});
    bd::Boid b3(vc::Vector{1., 1.}, vc::Vector{-1., -1.});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.updateVelocity();
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].getVelocity().getX() == doctest::Approx(0.8));
    CHECK(flk[0].getVelocity().getY() == doctest::Approx(-0.3));
    CHECK(flk[1].getVelocity().getX() == doctest::Approx(-1.35));
    CHECK(flk[1].getVelocity().getY() == doctest::Approx(-0.3));
    CHECK(flk[2].getVelocity().getX() == doctest::Approx(0.55));
    CHECK(flk[2].getVelocity().getY() == doctest::Approx(1.6));
  }
  SUBCASE("testing updatepositions with one boid") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    flock.addBoid(b1);
    flock.updatePosition(100);
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].getPosition().getX() == doctest::Approx(1));
    CHECK(flk[0].getPosition().getY() == doctest::Approx(0.1));
  }
  SUBCASE("testing updatepositions with distant boids (inside the border)") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1., 1.});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.updatePosition(300);
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].getPosition().getX() == doctest::Approx(1));
    CHECK(flk[0].getPosition().getY() == doctest::Approx(0.1));
    CHECK(flk[1].getPosition().getX() == doctest::Approx(100.1));
    CHECK(flk[1].getPosition().getY() == doctest::Approx(0.1));
  }
  SUBCASE("testing updatepositions with distant boids (outside the border)") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1., 1.});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.updatePosition(200);
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].getPosition().getX() == doctest::Approx(1));
    CHECK(flk[0].getPosition().getY() == doctest::Approx(0.1));
    CHECK(flk[1].getPosition().getX() == doctest::Approx(-100.0));
    CHECK(flk[1].getPosition().getY() == doctest::Approx(0.1));
  }
  SUBCASE("testing updatepositions with two distant boids and one close boid") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1., 1.});
    bd::Boid b3(vc::Vector{0., 0.}, vc::Vector{1., 1.});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.updatePosition(300);
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].getPosition().getX() == doctest::Approx(1.09));
    CHECK(flk[0].getPosition().getY() == doctest::Approx(0.1));
    CHECK(flk[1].getPosition().getX() == doctest::Approx(100.1));
    CHECK(flk[1].getPosition().getY() == doctest::Approx(0.1));
    CHECK(flk[2].getPosition().getX() == doctest::Approx(0.01));
    CHECK(flk[2].getPosition().getY() == doctest::Approx(0.1));
  }
  SUBCASE("testing updatepositions with 3 boids") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{0., 0.}, vc::Vector{1., 1.});
    bd::Boid b3(vc::Vector{1., 1.}, vc::Vector{-1., -1.});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.updatePosition(100);
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].getPosition().getX() == doctest::Approx(1.08));
    CHECK(flk[0].getPosition().getY() == doctest::Approx(-0.03));
    CHECK(flk[1].getPosition().getX() == doctest::Approx(-0.135));
    CHECK(flk[1].getPosition().getY() == doctest::Approx(-0.03));
    CHECK(flk[2].getPosition().getX() == doctest::Approx(1.055));
    CHECK(flk[2].getPosition().getY() == doctest::Approx(1.16));
  }
  SUBCASE("testing updateBoidParameters") {
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 1.}, vc::Vector{-1., -1.});
    flock.addBoid(b1);
    flock.addBoid(b2);
    const std::vector<bd::Boid> &flk = flock.getBoids();
    CHECK(flk[0].perceptionRadius == doctest::Approx(15.));
    CHECK(flk[0].separationDistance == doctest::Approx(2.));
    CHECK(flk[0].separationFactor == doctest::Approx(1.2));
    CHECK(flk[0].cohesionFactor == doctest::Approx(0.8));
    CHECK(flk[0].alignmentFactor == doctest::Approx(0.5));
    CHECK(flk[1].perceptionRadius == doctest::Approx(15.));
    CHECK(flk[1].separationDistance == doctest::Approx(2.));
    CHECK(flk[1].separationFactor == doctest::Approx(1.2));
    CHECK(flk[1].cohesionFactor == doctest::Approx(0.8));
    CHECK(flk[1].alignmentFactor == doctest::Approx(0.5));
    flock.updateBoidParameters(12., 3., 2., 0.25, 1.);
    CHECK(flk[0].perceptionRadius == doctest::Approx(12.));
    CHECK(flk[0].separationDistance == doctest::Approx(3.));
    CHECK(flk[0].separationFactor == doctest::Approx(2.));
    CHECK(flk[0].cohesionFactor == doctest::Approx(0.25));
    CHECK(flk[0].alignmentFactor == doctest::Approx(1.));
    CHECK(flk[1].perceptionRadius == doctest::Approx(12.));
    CHECK(flk[1].separationDistance == doctest::Approx(3.));
    CHECK(flk[1].separationFactor == doctest::Approx(2.));
    CHECK(flk[1].cohesionFactor == doctest::Approx(0.25));
    CHECK(flk[1].alignmentFactor == doctest::Approx(1.));
  }
  SUBCASE("testing average distance and its standard deviation with no boids") {
    double dist = flock.averageDistance();
    CHECK(dist == doctest::Approx(0.0));
    double stdDev = flock.standardDeviationDistance();
    CHECK(stdDev == doctest::Approx(0.0));
  }
  SUBCASE("testing average distance and its standard deviation with one boid") {
    bd::Boid b1(vc::Vector{1.0, 0.0});
    flock.addBoid(b1);
    double dist = flock.averageDistance();
    CHECK(dist == doctest::Approx(0.0));
    double stdDev = flock.standardDeviationDistance();
    CHECK(stdDev == doctest::Approx(0.0));
  }
  SUBCASE("testing average distance and its standard deviation") {
    bd::Boid b1(vc::Vector{1.0, 0.0});
    bd::Boid b2(vc::Vector{5.0, 1.0});
    bd::Boid b3(vc::Vector{3.0, 2.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    double dist = flock.averageDistance();
    CHECK(dist == doctest::Approx(3.0625));
    double stdDev = flock.standardDeviationDistance();
    CHECK(stdDev == doctest::Approx(0.814743));
  }
  SUBCASE("testing average speed and its standard deviation with no boids") {
    double m = flock.averageSpeed();
    CHECK(m == doctest::Approx(0.0));
    double s = flock.standardDeviationSpeed();
    CHECK(s == doctest::Approx(0.0));
  }
  SUBCASE("testing average speed and its standard deviation with one boid") {
    bd::Boid b1(vc::Vector{0.0, 0.0}, vc::Vector{1.0, 0.0});
    flock.addBoid(b1);
    double m = flock.averageSpeed();
    CHECK(m == doctest::Approx(1.0));
    double s = flock.standardDeviationSpeed();
    CHECK(s == doctest::Approx(0.0));
  }
  SUBCASE("testing average speed and its standard deviation") {
    bd::Boid b1(vc::Vector{0.0, 0.0}, vc::Vector{1.0, 0.0});
    bd::Boid b2(vc::Vector{0.0, 0.0}, vc::Vector{5.0, 1.0});
    bd::Boid b3(vc::Vector{0.0, 0.0}, vc::Vector{3.0, 2.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    double m = flock.averageSpeed();
    CHECK(m == doctest::Approx(3.23486));
    double s = flock.standardDeviationSpeed();
    CHECK(s == doctest::Approx(1.69382));
  }
  SUBCASE("testing countBoidsInFlock with 1 boid") {
    bd::Boid b1(vc::Vector{0.0, 0.0});
    flock.addBoid(b1);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius).size() == 1);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[0] == 1);
  }
  SUBCASE("testing countBoidsInFlock with 3 isolated boids") {
    bd::Boid b1(vc::Vector{0.0, 0.0});
    bd::Boid b2(vc::Vector{50.0, 0.0});
    bd::Boid b3(vc::Vector{100.0, 0.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius).size() == 3);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[0] == 1);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[1] == 1);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[2] == 1);
  }
  SUBCASE(
      "testing countBoidsInFlock with 3 close boids but last and first are "
      "distant") {
    bd::Boid b1(vc::Vector{0.0, 0.0});
    bd::Boid b2(vc::Vector{10.0, 0.0});
    bd::Boid b3(vc::Vector{20.0, 0.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius).size() == 1);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[0] == 3);
  }
  SUBCASE(
      "testing countBoidsInFlock with four close boids but last and first are "
      "distant") {
    bd::Boid b1(vc::Vector{0.0, 0.0});
    bd::Boid b2(vc::Vector{10.0, 0.0});
    bd::Boid b3(vc::Vector{20.0, 0.0});
    bd::Boid b4(vc::Vector{30.0, 0.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius).size() == 1);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[0] == 4);
  }
  SUBCASE(
      "testing countBoidsInFlock with three close boids and one isolated "
      "(first)") {
    bd::Boid b1(vc::Vector{0.0, 0.0});
    bd::Boid b2(vc::Vector{20.0, 0.0});
    bd::Boid b3(vc::Vector{30.0, 0.0});
    bd::Boid b4(vc::Vector{40.0, 0.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius).size() == 2);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[0] == 1);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[1] == 3);
  }
  SUBCASE(
      "testing countBoidsInFlock with three close boids and one isolated "
      "last)") {
    bd::Boid b1(vc::Vector{20.0, 0.0});
    bd::Boid b2(vc::Vector{30.0, 0.0});
    bd::Boid b3(vc::Vector{40.0, 0.0});
    bd::Boid b4(vc::Vector{0.0, 0.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius).size() == 2);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[0] == 3);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[1] == 1);
  }
  SUBCASE("testing countBoidsInFlock with two flocks") {
    bd::Boid b1(vc::Vector{0.0, 0.0});
    bd::Boid b2(vc::Vector{10.0, 0.0});
    bd::Boid b3(vc::Vector{20.0, 0.0});
    bd::Boid b4(vc::Vector{100.0, 0.0});
    bd::Boid b5(vc::Vector{110.0, 0.0});
    bd::Boid b6(vc::Vector{120.0, 0.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.addBoid(b5);
    flock.addBoid(b6);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius).size() == 2);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[0] == 3);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[1] == 3);
  }
  SUBCASE("testing countBoidsInFlock with two flocks but alternated") {
    bd::Boid b1(vc::Vector{0.0, 0.0});
    bd::Boid b2(vc::Vector{10.0, 0.0});
    bd::Boid b3(vc::Vector{120.0, 0.0});
    bd::Boid b4(vc::Vector{100.0, 0.0});
    bd::Boid b5(vc::Vector{110.0, 0.0});
    bd::Boid b6(vc::Vector{20.0, 0.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.addBoid(b5);
    flock.addBoid(b6);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius).size() == 2);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[0] == 3);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[1] == 3);
  }
  SUBCASE("testing countBoidsInFlock with two flocks but alternated") {
    bd::Boid b1(vc::Vector{0.0, 0.0});
    bd::Boid b2(vc::Vector{10.0, 0.0});
    bd::Boid b3(vc::Vector{120.0, 0.0});
    bd::Boid b4(vc::Vector{110.0, 0.0});
    bd::Boid b5(vc::Vector{20.0, 0.0});
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);
    flock.addBoid(b5);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius).size() == 2);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[0] == 3);
    CHECK(flock.countBoidsInFlock(b1.perceptionRadius)[1] == 2);
  }
}