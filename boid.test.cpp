#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boid.hpp"
TEST_CASE("Boid Rule Tests")
{
  Boid b1(Vector(1., 0.), Vector(0., 1.));//necessary to access class istance boids

  SUBCASE("Test separation rule")
  {
    b1.boids.push_back(Boid(Vector(0.0, 0.0), Vector(1.0, 1.0)));
    b1.boids.push_back(Boid(Vector(2.0, 2.0), Vector(-1.0, -1.0)));
    Vector separationVel1 = b1.separate();
    CHECK(separationVel1 == Vector(0.0, -2.0));
    //special cases to add
  }

  SUBCASE("Test cohesion rule")
  {
    b1.boids.push_back(Boid(Vector(0.0, 0.0), Vector(1.0, 1.0)));
    b1.boids.push_back(Boid(Vector(2.0, 2.0), Vector(-1.0, -1.0)));
    Vector cohesionVel1 = b1.cohere();
    CHECK(cohesionVel1 == Vector(0.0, 1.0));
    //special cases to add 
  }

  SUBCASE("Test alignment rule")
  {
    b1.boids.push_back(Boid(Vector(0.0, 0.0), Vector(1.0, 1.0)));
    b1.boids.push_back(Boid(Vector(2.0, 2.0), Vector(-1.0, -1.0)));
    Vector alignmentVel1 = b1.align();
    CHECK(alignmentVel1 == Vector(0.0, -1.0));
    //special cases to add
  }
}
TEST_CASE("testing methods")
{
  SUBCASE("setPosition")
  {
    Boid b;
    b.setPosition(1.0, 2.0);
    CHECK(b.pos() == Vector(1.0, 2.0));
  }
  SUBCASE("setVelocity")
  {
    Boid b;
    b.setVelocity(2.0, 3.0);
    CHECK(b.vel() == Vector(2.0, 3.0));
  }
  SUBCASE("center of mass with empty boids")
  {
    Boid b;
    CHECK(b.centerOfMass() == Vector(0.0, 0.0));
  }
  SUBCASE("center of mass")
  {
    Boid testBoid(Vector(1.0, 0.0));
    testBoid.boids.push_back(Boid(Vector(1.0, 1.0)));
    testBoid.boids.push_back(Boid(Vector(2.0, 2.0)));
    Vector center = testBoid.centerOfMass();
    CHECK(center == Vector(1.5, 1.5));
  }
  SUBCASE("speed")
  {
    Boid b(Vector(1.0, 3.0), Vector(3.0, 4.0));
    CHECK(b.speed() == doctest::Approx(5.0));
  }
  SUBCASE("update")
  {
    // Leo's should be fine.
  }
}
