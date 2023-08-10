#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"

#include "doctest.h"

TEST_CASE("Testing the constructors") {
  SUBCASE("default constructor") {
    Boid b1{};
    Boid b2{};
    CHECK(b1.getPosition().x == doctest::Approx(0.));
    CHECK(b1.getPosition().y == doctest::Approx(0.));
    CHECK(b2.getPosition().x == doctest::Approx(0.));
    CHECK(b2.getPosition().y == doctest::Approx(0.));
    CHECK(b1.getSpeed() > doctest::Approx(0.));
    CHECK(b2.getSpeed() > doctest::Approx(0.));
    CHECK(b1.getSpeed() < doctest::Approx(5.));
    CHECK(b2.getSpeed() < doctest::Approx(5.));
  }
  SUBCASE("Constructor with coordinates") {
    Boid b1{3., -1.};
    Boid b2{5., 0.};
    CHECK(b1.getPosition().x == doctest::Approx(3.));
    CHECK(b1.getPosition().y == doctest::Approx(-1.));
    CHECK(b2.getPosition().x == doctest::Approx(5.));
    CHECK(b2.getPosition().y == doctest::Approx(0.));
    CHECK(b1.getSpeed() > doctest::Approx(0.));
    CHECK(b2.getSpeed() > doctest::Approx(0.));
    CHECK(b1.getSpeed() < doctest::Approx(5.));
    CHECK(b2.getSpeed() < doctest::Approx(5.));
  }
  SUBCASE("Constructor with vector") {
    Vector v1{-2., -4.};
    Boid b1{v1};
    Boid b2{v1 * -0.5};
    CHECK(b1.getPosition().x == doctest::Approx(-2.));
    CHECK(b1.getPosition().y == doctest::Approx(-4.));
    CHECK(b2.getPosition().x == doctest::Approx(1.));
    CHECK(b2.getPosition().y == doctest::Approx(2.));
    CHECK(b1.getSpeed() > doctest::Approx(0.));
    CHECK(b2.getSpeed() > doctest::Approx(0.));
    CHECK(b1.getSpeed() < doctest::Approx(5.));
    CHECK(b2.getSpeed() < doctest::Approx(5.));
  }
  SUBCASE("Constructor with two vectors") {
    Boid b1{Vector(0., 0.), Vector(3., 2.)};
    Boid b2{Vector(2., 0.), Vector(0., 4.)};
    CHECK(b1.getPosition().x == doctest::Approx(0.));
    CHECK(b1.getPosition().y == doctest::Approx(0.));
    CHECK(b1.getVelocity().x == doctest::Approx(3.));
    CHECK(b1.getVelocity().y == doctest::Approx(2.));
    CHECK(b2.getPosition().x == doctest::Approx(2.));
    CHECK(b2.getPosition().y == doctest::Approx(0.));
    CHECK(b2.getVelocity().x == doctest::Approx(0.));
    CHECK(b2.getVelocity().y == doctest::Approx(4.));
  }
}
TEST_CASE("Testing the boid functions") {
  SUBCASE("Set functions") {
    Boid b1{3., -6.};
    b1.setPosition(3., -3.);
    b1.setVelocity(3., 4.);
    CHECK(b1.getPosition().x == doctest::Approx(3.));
    CHECK(b1.getPosition().y == doctest::Approx(-3.));
    b1.setPosition(0., 100.4563);
    CHECK(b1.getPosition().x == doctest::Approx(0.));
    CHECK(b1.getPosition().y == doctest::Approx(100.4563));
    CHECK(b1.getSpeed() == doctest::Approx(5.));
    b1.setVelocity(-54., 34.);
    CHECK(b1.getSpeed() == doctest::Approx(5.));
  }
  SUBCASE("Align function") {
    // testing using perceptionRadius = 10. , alignFactor = 0.5
    std::vector<Boid> boids;
    Boid b1{Vector(1., 0.), Vector(3., 2.)};
    Boid b2{Vector(2., 0.), Vector(0., 4.)};
    boids.push_back(b1);
    boids.push_back(b2);
    CHECK(b1.align(boids).x == doctest::Approx(-1.5));
    CHECK(b1.align(boids).y == doctest::Approx(1.));
    CHECK(b2.align(boids).x == doctest::Approx(1.5));
    CHECK(b2.align(boids).y == doctest::Approx(-1.));

    // with only one vector the rules don't activate
    boids.clear();
    boids.push_back(b1);
    CHECK(b1.align(boids).x == doctest::Approx(0.));
    CHECK(b1.align(boids).y == doctest::Approx(0.));

    // b2 its too far so align should be a null vector but it doesnt work in b1.align
    boids.push_back(b2);
    b1.setPosition(1., 0.);
    b1.setVelocity(0., 0.);
    b2.setPosition(15., 0.);
    b2.setVelocity(1., 1.);
    CHECK(b1.align(boids).x == doctest::Approx(0.));
    CHECK(b1.align(boids).y == doctest::Approx(0.));
    CHECK(b2.align(boids).x == doctest::Approx(0.));
    CHECK(b2.align(boids).y == doctest::Approx(0.));

// now it should activate because they are considered neighbors
    b1.setPosition(0., 0.);
    b1.setVelocity(0., 0.);
    b2.setPosition(2., 0.);
    b2.setVelocity(2., 0.);
    CHECK(b1.align(boids).x == doctest::Approx(??));
    CHECK(b1.align(boids).y == doctest::Approx(??));

/*

    boids.clear();
    Boid b5{Vector(0, 0), Vector(1, 1)};
    Boid b6{Vector(2, 0), Vector(-1, 1)};
    boids.push_back(b5);
    boids.push_back(b6);
    b5.update(boids);

    CHECK(b5.getVelocity().x == doctest::Approx(-1.267766953));
    CHECK(b5.getVelocity().y == doctest::Approx(2.267766953));
  */}
SUBCASE("Cohesion function") {
  // perceptionRadius is 10, cohesionFactor is 1
  // here align() and separate() don't activate
  std::vector<Boid> boids;
  Boid b1{Vector(0., 0.), Vector(1., 1.)};
  Boid b2{Vector(0., 2.), Vector(0., 2.)};
  boids.push_back(b1);
  boids.push_back(b2);
  CHECK(b1.cohere(boids).x == doctest::Approx(0.));
  CHECK(b1.cohere(boids).y == doctest::Approx(2.));
  CHECK(b2.cohere(boids).x == doctest::Approx(0.));
  CHECK(b2.cohere(boids).y == doctest::Approx(-2.));

//rules shouldn't activate because boids are too far
  b2.setPosition(80., 2.);
  CHECK(b2.cohere(boids).x == doctest::Approx(0.));
  CHECK(b2.cohere(boids).y == doctest::Approx(0.));
  CHECK(b1.cohere(boids).x == doctest::Approx(0.));
  CHECK(b1.cohere(boids).y == doctest::Approx(0.));
}
SUBCASE("Separate function") {
  // perceptionRadius is 10, separationFactor is 1, separation distance is 1.
  // here align() and cohere() don't activate
}
}