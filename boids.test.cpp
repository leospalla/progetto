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

    // this works without the boids[0] = ... syntax
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

    // b2 its too far so align should be a null vector but it doesnt work in
    // b1.align
    boids.push_back(b2);

    b1.setPosition(1., 0.);
    b1.setVelocity(0., 0.);
    b2.setPosition(15., 0.);
    b2.setVelocity(1., 1.);
    // now that i used the set functions, i modified from the outside the boids
    // so i need to use the boids[..] = ... syntax
    boids[0] = b1;
    boids[1] = b2;
    CHECK(b1.align(boids).x == doctest::Approx(0.));
    CHECK(b1.align(boids).y == doctest::Approx(0.));
    CHECK(b2.align(boids).x == doctest::Approx(0.));
    CHECK(b2.align(boids).y == doctest::Approx(0.));

    // now it should activate because they are considered neighbors

    // here instead if i use the set functions inside the container
    // everything goes smoothly because its all done from the inside
    boids[0].setPosition(0., 0.);
    boids[0].setVelocity(0., 0.);
    boids[1].setPosition(2., 0.);
    boids[1].setVelocity(2., 0.);
    CHECK(boids[0].align(boids).x == doctest::Approx(1.));
    CHECK(boids[0].align(boids).y == doctest::Approx(0.));

    // here i tried another basic test where i don't use the set functions
    // but i directly create new boids. this was to show that without using
    // the set functions the boids[.] = .. syntax isn't necessary
    boids.clear();
    Boid b3{Vector(0, 0), Vector(1, 1)};
    Boid b4{Vector(2, 0), Vector(-1, 1)};
    boids.push_back(b3);
    boids.push_back(b4);
    CHECK(b3.align(boids).x == doctest::Approx(-1.));
    CHECK(b3.align(boids).y == doctest::Approx(0.));
  }
  SUBCASE("Cohesion function") {
    // perceptionRadius is 10, cohesionFactor is 1
    std::vector<Boid> boids;
    Boid b1{Vector(0., 0.), Vector(1., 1.)};
    Boid b2{Vector(0., 2.), Vector(0., 2.)};
    boids.push_back(b1);
    boids.push_back(b2);
    CHECK(b1.cohere(boids).x == doctest::Approx(0.));
    CHECK(b1.cohere(boids).y == doctest::Approx(2.));
    CHECK(b2.cohere(boids).x == doctest::Approx(0.));
    CHECK(b2.cohere(boids).y == doctest::Approx(-2.));

    // rules shouldn't activate because boids are too far
    b2.setPosition(80., 2.);
    boids[1] = b2;
    CHECK(b2.cohere(boids).x == doctest::Approx(0.));
    CHECK(b2.cohere(boids).y == doctest::Approx(0.));
    CHECK(b1.cohere(boids).x == doctest::Approx(0.));
    CHECK(b1.cohere(boids).y == doctest::Approx(0.));
  }
  SUBCASE("Separate function") {
    // perceptionRadius is 10, separationFactor is 1, separation distance is 1.
    std::vector<Boid> boids;

    // boids are distant
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    boids.push_back(Boid(Vector(100., 0.), Vector(1.0, 1.0)));
    CHECK(b1.separate(boids).x == doctest::Approx(0.));
    CHECK(b1.separate(boids).y == doctest::Approx(0.));

    //need to add more tests
  }
}