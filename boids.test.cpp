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
}
TEST_CASE("Testing the boid functions") {
std::vector<Boid> boids;
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
    // testing using perceptionRadius = 10. , alignFactor = 1.
    Boid b1{};
    Boid b2{100., 0.};
    b1.setVelocity(1., 0.);
    b2.setVelocity(1., 1.);
    CHECK(boids.size() == doctest::Approx(0));
    b1.update();  // only adds b1's velocity
    CHECK(b1.getAcceleration().x == doctest::Approx(0.));
    CHECK(b1.getAcceleration().y == doctest::Approx(0.));
    CHECK(b1.getVelocity().x == doctest::Approx(1.));
    CHECK(b1.getVelocity().y == doctest::Approx(0.));
    CHECK(b1.getPosition().x == doctest::Approx(1.));
    CHECK(b1.getPosition().y == doctest::Approx(0.));
    boids.push_back(b1);
    boids.push_back(b2);
    CHECK(boids.size() == doctest::Approx(2));
    b1.update();
    // rules shouldn't activate bc b2 its too far
    CHECK(b1.getAcceleration().x == doctest::Approx(0.));
    CHECK(b1.getAcceleration().y == doctest::Approx(0.));
    CHECK(b1.getVelocity().x == doctest::Approx(1.));
    CHECK(b1.getVelocity().y == doctest::Approx(0.));
    CHECK(b1.getPosition().x == doctest::Approx(2.));
    CHECK(b1.getPosition().y == doctest::Approx(0.));
    b1.setPosition(0., 0.);
    b1.setVelocity(1., 1.);
    b2.setPosition(2., 0.);
    b2.setVelocity(-1., 1.);
    b1.update();
    b2.update();
    CHECK(b1.getAcceleration().x == doctest::Approx(0.));
    CHECK(b1.getAcceleration().y == doctest::Approx(0.));
    CHECK(b2.getAcceleration().x == doctest::Approx(0.));
    CHECK(b2.getAcceleration().y == doctest::Approx(0.));
    
    CHECK(b1.getVelocity().x == doctest::Approx(3.439818639));
    CHECK(b1.getVelocity().y == doctest::Approx(-3.364319536));


    Vector prova = b1.align();
    CHECK(prova.x == doctest::Approx(2.439818639));
    CHECK(prova.y == doctest::Approx(-4.364319536));

  }
}
