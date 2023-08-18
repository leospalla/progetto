#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boid.hpp"
TEST_CASE("Boid Rule Tests")
{

  SUBCASE("separation with empty boids")
  {
    std::vector<Boid> boids;
    Boid b;
    CHECK(b.separate(boids) == Vector(0.0, 0.0));
  }
  SUBCASE("separation with one boid")
  {
    std::vector<Boid> boids;
    Boid b(Vector(1., 0.), Vector(0., 1.));
    boids.push_back(b);
    CHECK(b.separate(boids) == Vector(0.0, 0.0));
  }
  SUBCASE("separation with distant boids")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(100., 0.), Vector(1., 1.));
    boids.push_back(b1);
    boids.push_back(b2);
    Vector separationVel1 = b1.separate(boids);
    Vector separationVel2 = b2.separate(boids);
    CHECK(separationVel1 == Vector(0.0, 0.0));
    CHECK(separationVel2 == Vector(0.0, 0.0));
  }
  SUBCASE("separation with one distant and one close boid")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(100., 0.), Vector(1.0, 1.0));
    Boid b3(Vector(0.0, 0.0), Vector(1.0, 1.0));
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    Vector separationVel1 = b1.separate(boids);
    Vector separationVel2 = b2.separate(boids);
    Vector separationVel3 = b3.separate(boids);
    CHECK(separationVel1 == Vector(1.0, 0.0));
    CHECK(separationVel2 == Vector(0.0, 0.0));
    CHECK(separationVel3 == Vector(-1.0, 0.0));
  }
  SUBCASE("separation with three boids")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(0.0, 0.0), Vector(1.0, 1.0));
    Boid b3(Vector(2.0, 2.0), Vector(-1.0, -1.0));
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    Vector separationVel1 = b1.separate(boids);
    Vector separationVel2 = b2.separate(boids);
    Vector separationVel3 = b3.separate(boids);
    CHECK(separationVel1 == Vector(0.0, -2.0));
    CHECK(separationVel2 == Vector(-3.0, -2.0));
    CHECK(separationVel3 == Vector(3.0, 4.0));
  }
  SUBCASE("cohesion with empty boids")
  {
    std::vector<Boid> boids;
    Boid b;
    CHECK(b.cohere(boids) == Vector(0.0, 0.0));
  }
  SUBCASE("cohesion with one boid")
  {
    std::vector<Boid> boids;
    Boid b(Vector(1., 0.), Vector(0., 1.));
    boids.push_back(b);
    CHECK(b.cohere(boids) == Vector(0.0, 0.0));
  }
  SUBCASE("cohesion with distant boids")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(100., 0.), Vector(1.0, 1.0));
    boids.push_back(b1);
    boids.push_back(b2);
    Vector cohesionVel1 = b1.cohere(boids);
    Vector cohesionVel2 = b2.cohere(boids);
    CHECK(cohesionVel1 == Vector(0.0, 0.0));
    CHECK(cohesionVel2 == Vector(0.0, 0.0));
  }
  SUBCASE("cohesion with one distant and one close boid")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(100., 0.), Vector(1.0, 1.0));
    Boid b3(Vector(0.0, 0.0), Vector(1.0, 1.0));
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    Vector cohesionVel1 = b1.cohere(boids);
    Vector cohesionVel2 = b2.cohere(boids);
    Vector cohesionVel3 = b3.cohere(boids);
    CHECK(cohesionVel1 == Vector(-1.0, 0.0));
    CHECK(cohesionVel2 == Vector(0.0, 0.0));
    CHECK(cohesionVel3 == Vector(1.0, 0.0));
  }
  SUBCASE("cohesion with three boids")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(0.0, 0.0), Vector(1.0, 1.0));
    Boid b3(Vector(2.0, 2.0), Vector(-1.0, -1.0));
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    Vector cohesionVel1 = b1.cohere(boids);
    Vector cohesionVel2 = b2.cohere(boids);
    Vector cohesionVel3 = b3.cohere(boids);
    CHECK(cohesionVel1 == Vector(0.0, 1.0));
    CHECK(cohesionVel2 == Vector(1.5, 1.0));
    CHECK(cohesionVel3 == Vector(-1.5, -2.0));
  }
  SUBCASE("alignment with empty boids")
  {
    std::vector<Boid> boids;
    Boid b;
    CHECK(b.align(boids) == Vector(0.0, 0.0));
  }
  SUBCASE("alignment with one boid")
  {
    std::vector<Boid> boids;
    Boid b(Vector(1., 0.), Vector(0., 1.));
    boids.push_back(b);
    CHECK(b.align(boids) == Vector(0.0, 0.0));
  }
  SUBCASE("alignment with distant boids")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(100., 0.), Vector(1.0, 1.0));
    boids.push_back(b1);
    boids.push_back(b2);
    Vector alignmentVel1 = b1.align(boids);
    Vector alignmentVel2 = b2.align(boids);
    CHECK(alignmentVel1 == Vector(0.0, 0.0));
    CHECK(alignmentVel2 == Vector(0.0, 0.0));
  }
  SUBCASE("alignment with one distant and one close boid")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(100., 0.), Vector(1.0, 1.0));
    Boid b3(Vector(0.0, 0.0), Vector(1.0, 0.0));
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    Vector alignmentVel1 = b1.align(boids);
    Vector alignmentVel2 = b2.align(boids);
    Vector alignmentVel3 = b3.align(boids);
    CHECK(alignmentVel1 == Vector(1.0, -1.0));
    CHECK(alignmentVel2 == Vector(0.0, 0.0));
    CHECK(alignmentVel3 == Vector(-1.0, 1.0));
  }

  SUBCASE("alignment with three boids")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(0.0, 0.0), Vector(1.0, 1.0));
    Boid b3(Vector(2.0, 2.0), Vector(-1.0, -1.0));
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    Vector alignmentVel1 = b1.align(boids);
    Vector alignmentVel2 = b2.align(boids);
    Vector alignmentVel3 = b3.align(boids);
    CHECK(alignmentVel1 == Vector(0.0, -1.0));
    CHECK(alignmentVel2 == Vector(-1.5, -1.0));
    CHECK(alignmentVel3 == Vector(1.5, 2.0));
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
    std::vector<Boid> boids;
    Boid b;
    CHECK(b.centerOfMass(boids) == Vector(0.0, 0.0));
  }
  SUBCASE("center of mass with one boid")
  {
    std::vector<Boid> boids;
    Boid b(Vector(1., 0.), Vector(0., 1.));
    boids.push_back(b);
    CHECK(b.centerOfMass(boids) == Vector(1.0, 0.0));
  }
    SUBCASE("center of mass with distant boids")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(100., 0.), Vector(1.0, 1.0));
    boids.push_back(b1);
    boids.push_back(b2);
    Vector center1 = b1.centerOfMass(boids);
    Vector center2 = b2.centerOfMass(boids);
    CHECK(center1 == Vector(1.0, 0.0));
    CHECK(center2 == Vector(100.0, 0.0));
  }
  SUBCASE("center of mass with one distant and one close boid")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(100., 0.), Vector(1.0, 1.0));
    Boid b3(Vector(0.0, 0.0), Vector(1.0, 0.0));
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    Vector center1 = b1.centerOfMass(boids);
    Vector center2 = b2.centerOfMass(boids);
    Vector center3 = b3.centerOfMass(boids);
    CHECK(center1 == Vector(0.0, 0.0));
    CHECK(center2 == Vector(100.0, 0.0));
    CHECK(center3 == Vector(1.0, 0.0));
  }
  SUBCASE("center of mass with three boids")
  {
    std::vector<Boid> boids;
    Boid b1(Vector(1., 0.), Vector(0., 1.));
    Boid b2(Vector(0.0, 0.0), Vector(1.0, 1.0));
    Boid b3(Vector(2.0, 2.0), Vector(-1.0, -1.0));
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    Vector center1 = b1.centerOfMass(boids);
    Vector center2 = b2.centerOfMass(boids);
    Vector center3 = b3.centerOfMass(boids);
    CHECK(center1 == Vector(1.0, 1.0));
    CHECK(center2 == Vector(1.5, 1.0));
    CHECK(center3 == Vector(0.5, 0.0));
  }
  SUBCASE("speed")
  {
    Boid b(Vector(1.0, 3.0), Vector(3.0, 4.0));
    CHECK(b.speed() == doctest::Approx(5.0));
  }
  SUBCASE("Testing border if boids are in window")
  {
    Boid b(Vector(-56, 51));
    b.border(200, 200);
    CHECK(b.pos() == Vector(-56, 51));
  }
  SUBCASE("Testing Border With x position > width")
  {
    Boid b(Vector(126, 51));
    b.border(200, 200);
    CHECK(b.pos() == Vector(-100, 51));
  }
    SUBCASE("Testing Border With x position < -width")
  {
    Boid b(Vector(-121, 51));
    b.border(200, 200);
    CHECK(b.pos() == Vector(100, 51));
  }
    SUBCASE("Testing Border With y position > height")
  {
    Boid b(Vector(-56, 181));
    b.border(200, 200);
    CHECK(b.pos() == Vector(-56, -100));
  }
    SUBCASE("Testing Border With y position < -height")
  {
    Boid b(Vector(-56, -203));
    b.border(200, 200);
    CHECK(b.pos() == Vector(-56, 100));
  }
    SUBCASE("Testing Border with both coordinates outside the window")
  {
    Boid b(Vector(128, -251));
    b.border(200, 200);
    CHECK(b.pos() == Vector(-100, 100));
  }
}
