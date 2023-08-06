#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boid.hpp"
TEST_CASE("testing the rules")
{
  SUBCASE("separation") //fails
  {
    std::vector<Boid> boids;
    boids.push_back(Boid(Vector(0., 0.), Vector(1., 1.)));
    boids.push_back(Boid(Vector(2., 2.), Vector(-1., -1.)));
    Boid b1 = boids[0];
    Boid b2 = boids[1];
    Vector separationVel1 = b1.separate();
    Vector separationVel2 = b2.separate();
    CHECK(separationVel1 == Vector(-2., -2.));
    CHECK(separationVel2 == Vector(2., 2.));
  }
  SUBCASE("cohesion")//fails
  {
    std::vector<Boid> boids;
    boids.push_back(Boid(Vector(0., 0.), Vector(1., 1.)));
    boids.push_back(Boid(Vector(2., 2.), Vector(2., 2.)));
    Boid b1 = boids[0];
    Vector cohesionVel1 = b1.cohere();
    CHECK(cohesionVel1 == Vector(-1., -1.));
  }
  SUBCASE("alignment")
  {
    std::vector<Boid> boids;
    boids.push_back(Boid(Vector(0., 0.), Vector(1., 1.)));
    boids.push_back(Boid(Vector(2., 2.), Vector(-1., -1.)));
    Boid b1 = boids[0];
    Boid b2 = boids[1];
    Vector alignmentVel1 = b1.align();
    Vector alignmentVel2 = b2.align();
    CHECK(alignmentVel1 == Vector(0., 0.));
    CHECK(alignmentVel2 == Vector(0., 0.));
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
  SUBCASE("center of mass")//fails
  {
    std::vector<Boid> boids;
    boids.push_back(Boid(Vector(0.0, 0.0)));
    boids.push_back(Boid(Vector(2.0, 2.0)));
    Boid testBoid;
    Vector center = testBoid.centerOfMass();
    CHECK(center == Vector(2., 2.));
  }
  SUBCASE("speed")
  {
    Boid b(Vector(1.0, 3.0), Vector(3.0, 4.0));
    CHECK(b.speed() == doctest::Approx(5.0));
  }
  SUBCASE("update")
  {
    // its gonna be a macello
  }
}
