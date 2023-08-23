#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"

TEST_CASE("Testing the constructors") {
  SUBCASE("Default constructor") {
    bd::Boid b1{};
    bd::Boid b2{};
    CHECK(b1.getPosition().getX() < doctest::Approx(30.));
    CHECK(b1.getPosition().getY() < doctest::Approx(30.));
    CHECK(b1.getPosition().getX() > doctest::Approx(-30.));
    CHECK(b1.getPosition().getY() > doctest::Approx(-30.));
    CHECK(b1.getSpeed() > doctest::Approx(0.));
    CHECK(b2.getSpeed() > doctest::Approx(0.));
    CHECK(b1.getSpeed() < doctest::Approx(10.));
    CHECK(b2.getSpeed() < doctest::Approx(10.));
  }
  SUBCASE("Constructor with random position") {
    bd::Boid b1{100};
    bd::Boid b2{-4};
    CHECK(b1.getPosition().getX() < doctest::Approx(100.));
    CHECK(b1.getPosition().getY() < doctest::Approx(100.));
    CHECK(b1.getPosition().getX() > doctest::Approx(-100.));
    CHECK(b1.getPosition().getY() > doctest::Approx(-100.));
    CHECK(b2.getPosition().getX() < doctest::Approx(4.));
    CHECK(b2.getPosition().getY() < doctest::Approx(4.));
    CHECK(b2.getPosition().getX() > doctest::Approx(-4.));
    CHECK(b2.getPosition().getY() > doctest::Approx(-4.));
    CHECK(b1.getSpeed() > doctest::Approx(0.));
    CHECK(b2.getSpeed() > doctest::Approx(0.));
    CHECK(b1.getSpeed() < doctest::Approx(10.));
    CHECK(b2.getSpeed() < doctest::Approx(10.));
  }
  SUBCASE("Constructor with coordinates") {
    bd::Boid b1{3., -1.};
    bd::Boid b2{5., 0.};
    CHECK(b1.getPosition().getX() == doctest::Approx(3.));
    CHECK(b1.getPosition().getY() == doctest::Approx(-1.));
    CHECK(b2.getPosition().getX() == doctest::Approx(5.));
    CHECK(b2.getPosition().getY() == doctest::Approx(0.));
    CHECK(b1.getSpeed() > doctest::Approx(0.));
    CHECK(b2.getSpeed() > doctest::Approx(0.));
    CHECK(b1.getSpeed() < doctest::Approx(10.));
    CHECK(b2.getSpeed() < doctest::Approx(10.));
  }
  SUBCASE("Constructor with vector") {
    vc::Vector v1{-2., -4.};
    bd::Boid b1{v1};
    bd::Boid b2{v1 * -0.5};
    CHECK(b1.getPosition().getX() == doctest::Approx(-2.));
    CHECK(b1.getPosition().getY() == doctest::Approx(-4.));
    CHECK(b2.getPosition().getX() == doctest::Approx(1.));
    CHECK(b2.getPosition().getY() == doctest::Approx(2.));
    CHECK(b1.getSpeed() > doctest::Approx(0.));
    CHECK(b2.getSpeed() > doctest::Approx(0.));
    CHECK(b1.getSpeed() < doctest::Approx(10.));
    CHECK(b2.getSpeed() < doctest::Approx(10.));
  }
  SUBCASE("Constructor with two vectors") {
    bd::Boid b1{vc::Vector{0., 0.}, vc::Vector{3., 2.}};
    bd::Boid b2{vc::Vector{2., 0.}, vc::Vector{0., 4.}};
    CHECK(b1.getPosition().getX() == doctest::Approx(0.));
    CHECK(b1.getPosition().getY() == doctest::Approx(0.));
    CHECK(b1.getVelocity().getX() == doctest::Approx(3.));
    CHECK(b1.getVelocity().getY() == doctest::Approx(2.));
    CHECK(b2.getPosition().getX() == doctest::Approx(2.));
    CHECK(b2.getPosition().getY() == doctest::Approx(0.));
    CHECK(b2.getVelocity().getX() == doctest::Approx(0.));
    CHECK(b2.getVelocity().getY() == doctest::Approx(4.));
  }
}
TEST_CASE("Testing the Boid functions") {
  SUBCASE("Set position/velocity  and getSpeed functions") {
    bd::Boid b1{3., -6.};
    b1.setPosition(3., -3.);
    b1.setVelocity(3., 4.);
    CHECK(b1.getPosition().getX() == doctest::Approx(3.));
    CHECK(b1.getPosition().getY() == doctest::Approx(-3.));
    b1.setPosition(0., 100.4563);
    CHECK(b1.getPosition().getX() == doctest::Approx(0.));
    CHECK(b1.getPosition().getY() == doctest::Approx(100.4563));
    CHECK(b1.getSpeed() == doctest::Approx(5.));
    b1.setVelocity(-54., 34.);
    CHECK(b1.getSpeed() == doctest::Approx(10.));
  }
  SUBCASE("Set parameters functions") {
    bd::Boid b1{};
    CHECK(b1.perceptionRadius == doctest::Approx(15.));
    CHECK(b1.separationDistance == doctest::Approx(2.));
    CHECK(b1.separationFactor == doctest::Approx(1.2));
    CHECK(b1.cohesionFactor == doctest::Approx(0.8));
    CHECK(b1.alignmentFactor == doctest::Approx(0.5));
    b1.setPerceptionRadius(10.5);
    b1.setSeparationDistance(3.);
    b1.setSeparationFactor(2.);
    b1.setCohesionFactor(10.);
    b1.setAlignmentFactor(0.);
    CHECK(b1.perceptionRadius == doctest::Approx(10.5));
    CHECK(b1.separationDistance == doctest::Approx(3.));
    CHECK(b1.separationFactor == doctest::Approx(2.));
    CHECK(b1.cohesionFactor == doctest::Approx(10.));
    CHECK(b1.alignmentFactor == doctest::Approx(0.));
  }
  SUBCASE("separation with empty vector") {
    std::vector<bd::Boid> boids;
    bd::Boid b;
    CHECK(boids.size() == 0.);
    CHECK(b.separate(boids).getX() == doctest::Approx(0.));
    CHECK(b.separate(boids).getY() == doctest::Approx(0.));
  }
  SUBCASE("separation with one boid") {
    std::vector<bd::Boid> boids;
    bd::Boid b(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    boids.push_back(b);
    CHECK(b.separate(boids).getX() == doctest::Approx(0.));
    CHECK(b.separate(boids).getY() == doctest::Approx(0.));
  }
  SUBCASE("separation with distant boids") {
    std::vector<bd::Boid> boids;
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1., 1.});
    boids.push_back(b1);
    boids.push_back(b2);
    vc::Vector separationVel1 = b1.separate(boids);
    vc::Vector separationVel2 = b2.separate(boids);
    CHECK(separationVel1.getX() == doctest::Approx(0.));
    CHECK(separationVel1.getY() == doctest::Approx(0.));
    CHECK(separationVel2.getX() == doctest::Approx(0.));
    CHECK(separationVel2.getY() == doctest::Approx(0.));
  }
  SUBCASE("separation with one distant and one close boid") {
    std::vector<bd::Boid> boids;
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1.0, 1.0});
    bd::Boid b3(vc::Vector{0.0, 0.0}, vc::Vector{1.0, 1.0});
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    vc::Vector separationVel1 = b1.separate(boids);
    vc::Vector separationVel2 = b2.separate(boids);
    vc::Vector separationVel3 = b3.separate(boids);
    CHECK(separationVel1.getX() == doctest::Approx(1.2));
    CHECK(separationVel1.getY() == doctest::Approx(0.));
    CHECK(separationVel2.getX() == doctest::Approx(0.));
    CHECK(separationVel2.getY() == doctest::Approx(0.));
    CHECK(separationVel3.getX() == doctest::Approx(-1.2));
    CHECK(separationVel3.getY() == doctest::Approx(0.));
  }
  SUBCASE("separation with three boids") {
    std::vector<bd::Boid> boids;
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{0.0, 0.0}, vc::Vector{1.0, 1.0});
    bd::Boid b3(vc::Vector{1.0, 1.0}, vc::Vector{-1.0, -1.0});
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    vc::Vector separationVel1 = b1.separate(boids);
    vc::Vector separationVel2 = b2.separate(boids);
    vc::Vector separationVel3 = b3.separate(boids);
    CHECK(separationVel1.getX() == doctest::Approx(1.2));
    CHECK(separationVel1.getY() == doctest::Approx(-1.2));
    CHECK(separationVel2.getX() == doctest::Approx(-2.4));
    CHECK(separationVel2.getY() == doctest::Approx(-1.2));
    CHECK(separationVel3.getX() == doctest::Approx(1.2));
    CHECK(separationVel3.getY() == doctest::Approx(2.4));
  }
  SUBCASE("cohesion with empty vector") {
    std::vector<bd::Boid> boids;
    bd::Boid b;
    CHECK(boids.size() == 0.);
    CHECK(b.cohere(boids).getX() == doctest::Approx(0.));
    CHECK(b.cohere(boids).getY() == doctest::Approx(0.));
  }
  SUBCASE("cohesion with one boid") {
    std::vector<bd::Boid> boids;
    bd::Boid b(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    boids.push_back(b);
    CHECK(b.cohere(boids).getX() == doctest::Approx(0.));
    CHECK(b.cohere(boids).getY() == doctest::Approx(0.));
  }
  SUBCASE("cohesion with distant boids") {
    std::vector<bd::Boid> boids;
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1., 1.});
    boids.push_back(b1);
    boids.push_back(b2);
    vc::Vector cohesionVel1 = b1.cohere(boids);
    vc::Vector cohesionVel2 = b2.cohere(boids);
    CHECK(cohesionVel1.getX() == doctest::Approx(0.));
    CHECK(cohesionVel1.getY() == doctest::Approx(0.));
    CHECK(cohesionVel1.getX() == doctest::Approx(0.));
    CHECK(cohesionVel2.getY() == doctest::Approx(0.));
  }
  SUBCASE("cohesion with one distant and one close boid") {
    std::vector<bd::Boid> boids;
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1.0, 1.0});
    bd::Boid b3(vc::Vector{0.0, 0.0}, vc::Vector{1.0, 1.0});
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    vc::Vector cohesionVel1 = b1.cohere(boids);
    vc::Vector cohesionVel2 = b2.cohere(boids);
    vc::Vector cohesionVel3 = b3.cohere(boids);
    CHECK(cohesionVel1.getX() == doctest::Approx(-0.8));
    CHECK(cohesionVel1.getY() == doctest::Approx(0.));
    CHECK(cohesionVel2.getX() == doctest::Approx(0.));
    CHECK(cohesionVel2.getY() == doctest::Approx(0.));
    CHECK(cohesionVel3.getX() == doctest::Approx(0.8));
    CHECK(cohesionVel3.getY() == doctest::Approx(0.));
  }
  SUBCASE("cohesion with three boids") {
    std::vector<bd::Boid> boids;
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{0.0, 0.0}, vc::Vector{1.0, 1.0});
    bd::Boid b3(vc::Vector{1.0, 1.0}, vc::Vector{-1.0, -1.0});
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    vc::Vector cohesionVel1 = b1.cohere(boids);
    vc::Vector cohesionVel2 = b2.cohere(boids);
    vc::Vector cohesionVel3 = b3.cohere(boids);
    CHECK(cohesionVel1.getX() == doctest::Approx(-0.4));
    CHECK(cohesionVel1.getY() == doctest::Approx(0.4));
    CHECK(cohesionVel2.getX() == doctest::Approx(0.8));
    CHECK(cohesionVel2.getY() == doctest::Approx(0.4));
    CHECK(cohesionVel3.getX() == doctest::Approx(-0.4));
    CHECK(cohesionVel3.getY() == doctest::Approx(-0.8));
  }
  SUBCASE("alignment with empty vector") {
    std::vector<bd::Boid> boids;
    bd::Boid b;
    CHECK(boids.size() == 0.);
    CHECK(b.align(boids).getX() == doctest::Approx(0.));
    CHECK(b.align(boids).getY() == doctest::Approx(0.));
  }
  SUBCASE("alignment with one boid") {
    std::vector<bd::Boid> boids;
    bd::Boid b(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    boids.push_back(b);
    CHECK(b.align(boids).getX() == doctest::Approx(0.));
    CHECK(b.align(boids).getY() == doctest::Approx(0.));
  }
  SUBCASE("alignment with distant boids") {
    std::vector<bd::Boid> boids;
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1., 1.});
    boids.push_back(b1);
    boids.push_back(b2);
    vc::Vector alignmentVel1 = b1.align(boids);
    vc::Vector alignmentVel2 = b2.align(boids);
    CHECK(alignmentVel1.getX() == doctest::Approx(0.));
    CHECK(alignmentVel1.getY() == doctest::Approx(0.));
    CHECK(alignmentVel2.getX() == doctest::Approx(0.));
    CHECK(alignmentVel2.getY() == doctest::Approx(0.));
  }
  SUBCASE("alignment with one distant and one close boid") {
    std::vector<bd::Boid> boids;
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{100., 0.}, vc::Vector{1.0, 1.0});
    bd::Boid b3(vc::Vector{0.0, 0.0}, vc::Vector{1.0, 0.0});
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    vc::Vector alignmentVel1 = b1.align(boids);
    vc::Vector alignmentVel2 = b2.align(boids);
    vc::Vector alignmentVel3 = b3.align(boids);
    CHECK(alignmentVel1.getX() == doctest::Approx(0.5));
    CHECK(alignmentVel1.getY() == doctest::Approx(-0.5));
    CHECK(alignmentVel2.getX() == doctest::Approx(0.));
    CHECK(alignmentVel2.getY() == doctest::Approx(0.));
    CHECK(alignmentVel3.getX() == doctest::Approx(-0.5));
    CHECK(alignmentVel3.getY() == doctest::Approx(0.5));
  }

  SUBCASE("alignment with three boids") {
    std::vector<bd::Boid> boids;
    bd::Boid b1(vc::Vector{1., 0.}, vc::Vector{0., 1.});
    bd::Boid b2(vc::Vector{0.0, 0.0}, vc::Vector{1.0, 1.0});
    bd::Boid b3(vc::Vector{1.0, 1.0}, vc::Vector{-1.0, -1.0});
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    vc::Vector alignmentVel1 = b1.align(boids);
    vc::Vector alignmentVel2 = b2.align(boids);
    vc::Vector alignmentVel3 = b3.align(boids);
    CHECK(alignmentVel1.getX() == doctest::Approx(0.));
    CHECK(alignmentVel1.getY() == doctest::Approx(-0.5));
    CHECK(alignmentVel2.getX() == doctest::Approx(-0.75));
    CHECK(alignmentVel2.getY() == doctest::Approx(-0.5));
    CHECK(alignmentVel3.getX() == doctest::Approx(0.75));
    CHECK(alignmentVel3.getY() == doctest::Approx(1.));
  }
  SUBCASE("Testing borders if boids are in window") {
    bd::Boid b(vc::Vector{-56, 51});
    b.borders(200);
    CHECK(b.getPosition() == vc::Vector{-56, 51});
  }
  SUBCASE("Testing borders With x position > width") {
    bd::Boid b(vc::Vector{126, 51});
    b.borders(200);
    CHECK(b.getPosition() == vc::Vector{-100, 51});
  }
  SUBCASE("Testing borders With x position < -width") {
    bd::Boid b(vc::Vector{-121, 51});
    b.borders(200);
    CHECK(b.getPosition() == vc::Vector{100, 51});
  }
  SUBCASE("Testing borders With y position > height") {
    bd::Boid b(vc::Vector{-56, 181});
    b.borders(200);
    CHECK(b.getPosition() == vc::Vector{-56, -100});
  }
  SUBCASE("Testing borders With y position < -height") {
    bd::Boid b(vc::Vector{-56, -203});
    b.borders(200);
    CHECK(b.getPosition() == vc::Vector{-56, 100});
  }
  SUBCASE("Testing borders with both coordinates outside the window") {
    bd::Boid b(vc::Vector{128, -251});
    b.borders(200);
    CHECK(b.getPosition() == vc::Vector{-100, 100});
  }
}