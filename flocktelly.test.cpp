#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "flocktelly.hpp"

#include "doctest.h"

TEST_CASE("testing ops") {
  SUBCASE("testing add") {
    Boid b;
    Flock f;
    f.addBoid(b);
    CHECK(f.getnumboids() != 0);
  }
  SUBCASE("testing rem") {
    Boid a;
    Boid b;
    Flock f;
    f.getmboids().push_back(a);
    f.getmboids().push_back(b);
    f.removeBoid(a);
    CHECK(f.getmboids().size() == 1);
    CHECK(f.getmboids().at(0) == b);
  }
  SUBCASE("testing updatevel") {
    Boid a;
    Boid b;
    Boid c;
    a.setPosition(2., 2.);
    a.setVelocity(1., -2.);
    b.setPosition(3., 0.);
    b.setVelocity(0., -6);
    c.setPosition(4., 7.);
    c.setVelocity(4., 2.);
    Flock f;
    f.getmboids().push_back(a);
    f.getmboids().push_back(b);
    f.getmboids().push_back(c);
    f.updateVelocity();
    CHECK(a.separate(f.getmboids()) == Vector(0., 0.));

    // bisogna aggiungere altri test ma sono calcoli scemi credo
  }
  SUBCASE("testing update pos") {
    Boid a;
    Boid b;
    Boid c;

    a.setPosition(2., 2.);
    a.setVelocity(1., -2.);

    b.setPosition(3., 0.);
    b.setVelocity(0., -6);

    c.setPosition(4., 7.);
    c.setVelocity(4., 2.);

    Flock f;

    f.getmboids().push_back(a);
    f.getmboids().push_back(b);
    f.getmboids().push_back(c);
    f.updatePosition();
  }
  SUBCASE("TESTING DISTANZA MEDIA") {}
}