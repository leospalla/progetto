#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "flock.hpp"
TEST_CASE("testing methods")
{
    Flock flock;
    SUBCASE("testing add boid and remove boid")
    {
        Boid b1;
        Boid b2;
        flock.addBoid(b1);
        flock.addBoid(b2);
        const std::vector<Boid> &flk1 = flock.getBoids();
        CHECK(flk1.size() == 2.0);
        CHECK(flk1[0] == b1);
        CHECK(flk1[1] == b2);
        flock.removeBoid(b1);
        const std::vector<Boid> &flk2 = flock.getBoids();
        CHECK(flk2.size() == 1.0);
        CHECK(flk2[0] == b2);
    }
    SUBCASE("testing average distance and the standard deviation of the distance functions")
    {
        Boid b1(Vector(1.0, 0.0));
        Boid b2(Vector(5.0, 1.0));
        Boid b3(Vector(3.0, 2.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        double d = flock.averageDistance();
        CHECK(d == doctest::Approx(3.0625));
        double s = flock.standardDeviationDistance();
        CHECK(s == doctest::Approx(0.787965));
        //special case with one boid to add
    }
    SUBCASE("testing average speed and the standard deviation of the speed functions")
    {
        Boid b1(Vector(0.0, 0.0), Vector(1.0, 0.0));
        Boid b2(Vector(0.0, 0.0), Vector(5.0, 1.0));
        Boid b3(Vector(0.0, 0.0), Vector(3.0, 2.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        double d = flock.averageSpeed();
        CHECK(d == doctest::Approx(3.23486));
        double s = flock.standardDeviationSpeed();
        CHECK(s == doctest::Approx(1.69382));
        //special case with one boid to add
    }
    SUBCASE("testing updatevelocity")
    {
        Boid b1(Vector(1., 0.), Vector(0., 1.));
        Boid b2(Vector(0.0, 0.0), Vector(1.0, 1.0));
        Boid b3(Vector(2.0, 2.0), Vector(-1.0, -1.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        flock.updateVelocity();
        const std::vector<Boid> &flk = flock.getBoids();
        CHECK(flk[0].vel() == Vector(0.0, -1.0));
        CHECK(flk[1].vel() == Vector(-2.0, -1.0));
        CHECK(flk[2].vel() == Vector(2.0, 3.0));
        // we could add different special cases but i think they will pass because the rules are succesfull
    }
    SUBCASE("testing updateposition")
    {
        Boid b1(Vector(1., 0.), Vector(0., 1.));
        Boid b2(Vector(0.0, 0.0), Vector(1.0, 1.0));
        Boid b3(Vector(2.0, 2.0), Vector(-1.0, -1.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        flock.updatePosition();
        const std::vector<Boid> &flk = flock.getBoids();
        CHECK(flk[0].pos() == Vector(1.0, -1.0));
        CHECK(flk[1].pos() == Vector(-2.0, -1.0));
        CHECK(flk[2].pos() == Vector(4.0, 5.0));
    }
}