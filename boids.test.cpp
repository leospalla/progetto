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
    Boid b1{Vector(0., 0.), Vector(3., 2.)};
    Boid b2{Vector(2., 0.), Vector(0., 4.)};
    boids.push_back(b1);
    boids.push_back(b2);

    b1.update(boids);
    CHECK(boids.size() == doctest::Approx(2));
    CHECK(b1.getVelocity().x == doctest::Approx(1.5));
    CHECK(b1.getVelocity().y == doctest::Approx(3.5));
    CHECK(b1.getPosition().x == doctest::Approx(1.5));
    CHECK(b1.getPosition().y == doctest::Approx(3.5));

    boids.clear();
    CHECK(boids.size() == doctest::Approx(0));
    Boid b3{};
    Boid b4{100., 0.};
    b3.setVelocity(1., 0.);
    b4.setVelocity(1., 1.);
    b3.update(boids);  // only adds b1's velocity
    CHECK(b3.getAcceleration().x == doctest::Approx(0.));
    CHECK(b3.getAcceleration().y == doctest::Approx(0.));
    CHECK(b3.getVelocity().x == doctest::Approx(1.));
    CHECK(b3.getVelocity().y == doctest::Approx(0.));
    CHECK(b3.getPosition().x == doctest::Approx(1.));
    CHECK(b3.getPosition().y == doctest::Approx(0.));

    boids.push_back(b3);
    boids.push_back(b4);
    CHECK(boids.size() == doctest::Approx(2));
    b3.update(boids);
    // rules shouldn't activate bc b2 its too far
    CHECK(b3.getAcceleration().x == doctest::Approx(0.));
    CHECK(b3.getAcceleration().y == doctest::Approx(0.));
    CHECK(b3.getVelocity().x == doctest::Approx(1.));
    CHECK(b3.getVelocity().y == doctest::Approx(0.));
    CHECK(b3.getPosition().x == doctest::Approx(2.));
    CHECK(b3.getPosition().y == doctest::Approx(0.));

    // now it should activate because they are considered neighbors
    b3.setPosition(0., 0.);
    b3.setVelocity(0., 0.);
    b4.setPosition(2., 0.);
    b4.setVelocity(2., 0.);
    b4.update(boids);
    b3.update(boids);

    CHECK(b3.getAcceleration().x == doctest::Approx(0.));
    CHECK(b3.getAcceleration().y == doctest::Approx(0.));
    CHECK(b3.getVelocity().x == doctest::Approx(2.5));
    CHECK(b3.getVelocity().y == doctest::Approx(0.));
    CHECK(b3.getPosition().x == doctest::Approx(2.5));
    CHECK(b3.getPosition().y == doctest::Approx(0.));

    boids.clear();
    Boid b5{Vector(0,0), Vector(1,1)};
    Boid b6{Vector(2,0), Vector(-1,1)};
    boids.push_back(b5);
    boids.push_back(b6);
    b5.update(boids);

    CHECK(b5.getVelocity().x == doctest::Approx(-1.267766953));
    CHECK(b5.getVelocity().y == doctest::Approx(2.267766953));

    /*
    Boid b1{};
    Boid b2{100., 0.};
    b1.setVelocity(1., 0.);
    b2.setVelocity(1., 1.);
    CHECK(b1.boids.size() == doctest::Approx(0));
    b1.update();  // only adds b1's velocity
    CHECK(b1.getAcceleration().x == doctest::Approx(0.));
    CHECK(b1.getAcceleration().y == doctest::Approx(0.));
    CHECK(b1.getVelocity().x == doctest::Approx(1.));
    CHECK(b1.getVelocity().y == doctest::Approx(0.));
    CHECK(b1.getPosition().x == doctest::Approx(1.));
    CHECK(b1.getPosition().y == doctest::Approx(0.));

    b1.boids.push_back(b1);
    b1.boids.push_back(b2);
    CHECK(b1.boids.size() == doctest::Approx(2));
    b1.update();
    // rules shouldn't activate bc b2 its too far
    CHECK(b1.getAcceleration().x == doctest::Approx(0.));
    CHECK(b1.getAcceleration().y == doctest::Approx(0.));
    CHECK(b1.getVelocity().x == doctest::Approx(1.));
    CHECK(b1.getVelocity().y == doctest::Approx(0.));
    CHECK(b1.getPosition().x == doctest::Approx(2.));
    CHECK(b1.getPosition().y == doctest::Approx(0.));

    //now it should activate because they are considered neighbors
    b1.setPosition(0., 0.);
    b1.setVelocity(0., 0.);
    b2.setPosition(2., 0.);
    b2.setVelocity(2., 0.);
    b2.update();
    b1.update();

    CHECK(b1.getAcceleration().x == doctest::Approx(0.));
    CHECK(b1.getAcceleration().y == doctest::Approx(0.));
  //  CHECK(b1.getVelocity().x == doctest::Approx(2.5));
   // CHECK(b1.getVelocity().y == doctest::Approx(0.));
   // CHECK(b1.getPosition().x == doctest::Approx(2.5));
   // CHECK(b1.getPosition().y == doctest::Approx(0.));

    b1.setPosition(0., 0.);
    b1.setVelocity(3., 2.);
    b2.setPosition(2., 0.);
    b2.setVelocity(0., 4.);
    b1.update();
    //these dont work idk why
    //through debugging i found out that for some reason in the flow control of
  the align function, otherBoid.getVelocity() is a (4,2) vector
    CHECK(b1.boids[1].getVelocity().x == doctest::Approx(1.5));
    CHECK(b1.boids[1].getVelocity().y == doctest::Approx(3.5));
    CHECK(b1.boids[0].getPosition().x == doctest::Approx(1.5));
    CHECK(b1.boids[0].getPosition().y == doctest::Approx(3.5));


*/
  }
}
