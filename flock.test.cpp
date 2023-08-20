#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "flock.hpp"
TEST_CASE("testing methods")
{
    fk::Flock flock;
    SUBCASE("testing add boid and remove boid")
    {
        bd::Boid b1;
        bd::Boid b2;
        flock.addBoid(b1);
        flock.addBoid(b2);
        const std::vector<bd::Boid> &flk1 = flock.getBoids();
        CHECK(flk1.size() == 2.0);
        CHECK(flk1[0] == b1);
        CHECK(flk1[1] == b2);
        flock.removeBoid(b1);
        const std::vector<bd::Boid> &flk2 = flock.getBoids();
        CHECK(flk2.size() == 1.0);
        CHECK(flk2[0] == b2);
    }
    SUBCASE("testing average distance and the standard deviation of the distance functions with no boids")
    {
        double d = flock.averageDistance();
        CHECK(d == doctest::Approx(0.0));
        double s = flock.standardDeviationDistance();
        CHECK(s == doctest::Approx(0.0));
    }
    SUBCASE("testing average distance and the standard deviation of the distance functions with one boid")
    {
        bd::Boid b1(Vector(1.0, 0.0));
        flock.addBoid(b1);
        double d = flock.averageDistance();
        CHECK(d == doctest::Approx(0.0));
        double s = flock.standardDeviationDistance();
        CHECK(s == doctest::Approx(0.0));
    }
    SUBCASE("testing average distance and the standard deviation of the distance functions")
    {
        bd::Boid b1(Vector(1.0, 0.0));
        bd::Boid b2(Vector(5.0, 1.0));
        bd::Boid b3(Vector(3.0, 2.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        double d = flock.averageDistance();
        CHECK(d == doctest::Approx(3.0625));
        double s = flock.standardDeviationDistance();
        CHECK(s == doctest::Approx(0.787965));
    }
    SUBCASE("testing average speed and the standard deviation of the speed functions with no boids")
    {
        double m = flock.averageSpeed();
        CHECK(m == doctest::Approx(0.0));
        double s = flock.standardDeviationSpeed();
        CHECK(s == doctest::Approx(0.0));
    }
    SUBCASE("testing average speed and the standard deviation of the speed functions with one boid")
    {
        bd::Boid b1(Vector(0.0, 0.0), Vector(1.0, 0.0));
        flock.addBoid(b1);
        double m = flock.averageSpeed();
        CHECK(m == doctest::Approx(1.0));
        double s = flock.standardDeviationSpeed();
        CHECK(s == doctest::Approx(0.0));
    }
    SUBCASE("testing average speed and the standard deviation of the speed functions")
    {
        bd::Boid b1(Vector(0.0, 0.0), Vector(1.0, 0.0));
        bd::Boid b2(Vector(0.0, 0.0), Vector(5.0, 1.0));
        bd::Boid b3(Vector(0.0, 0.0), Vector(3.0, 2.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        double m = flock.averageSpeed();
        CHECK(m == doctest::Approx(3.23486));
        double s = flock.standardDeviationSpeed();
        CHECK(s == doctest::Approx(1.69382));
    }
    SUBCASE("testing updatevelocity with one boid")
    {
        bd::Boid b1(Vector(1., 0.), Vector(0., 1.));
        flock.addBoid(b1);
        flock.updateVelocity();
        const std::vector<bd::Boid> &flk = flock.getBoids();
        CHECK(flk[0].vel().xcomp() == doctest::Approx(0.0));
        CHECK(flk[0].vel().ycomp() == doctest::Approx(1.0));
    }
    SUBCASE("testing updatevelocity with distant boids")
    {
        bd::Boid b1(Vector(1., 0.), Vector(0., 1.));
        bd::Boid b2(Vector(100., 0.), Vector(1., 1.));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.updateVelocity();
        const std::vector<bd::Boid> &flk = flock.getBoids();
        CHECK(flk[0].vel().xcomp() == doctest::Approx(0.0));
        CHECK(flk[0].vel().ycomp() == doctest::Approx(1.0));
        CHECK(flk[1].vel().xcomp() == doctest::Approx(1.0));
        CHECK(flk[1].vel().ycomp() == doctest::Approx(1.0));
    }
    SUBCASE("testing updatevelocity with two distant boids and one close boid")
    {
        bd::Boid b1(Vector(1., 0.), Vector(0.0, 1.0));
        bd::Boid b2(Vector(100., 0.), Vector(1.0, 1.0));
        bd::Boid b3(Vector(0.0, 0.0), Vector(1.0, 1.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        flock.updateVelocity();
        const std::vector<bd::Boid> &flk = flock.getBoids();
        CHECK(flk[0].vel().xcomp() == doctest::Approx(0.9));
        CHECK(flk[0].vel().ycomp() == doctest::Approx(1.0));
        CHECK(flk[1].vel().xcomp() == doctest::Approx(1.0));
        CHECK(flk[1].vel().ycomp() == doctest::Approx(1.0));
        CHECK(flk[2].vel().xcomp() == doctest::Approx(0.1));
        CHECK(flk[2].vel().ycomp() == doctest::Approx(1.0));
    }
    SUBCASE("testing updatevelocity with 3 boids")
    {
        bd::Boid b1(Vector(1., 0.), Vector(0., 1.));
        bd::Boid b2(Vector(0.0, 0.0), Vector(1.0, 1.0));
        bd::Boid b3(Vector(1.0, 1.0), Vector(-1.0, -1.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        flock.updateVelocity();
        const std::vector<bd::Boid> &flk = flock.getBoids();
        CHECK(flk[0].vel().xcomp() == doctest::Approx(0.8));
        CHECK(flk[0].vel().ycomp() == doctest::Approx(-0.3));
        CHECK(flk[1].vel().xcomp() == doctest::Approx(-1.35));
        CHECK(flk[1].vel().ycomp() == doctest::Approx(-0.3));
        CHECK(flk[2].vel().xcomp() == doctest::Approx(0.55));
        CHECK(flk[2].vel().ycomp() == doctest::Approx(1.6));
    }
    SUBCASE("testing updatepositions with one boid")
    {
        bd::Boid b1(Vector(1., 0.), Vector(0., 1.));
        flock.addBoid(b1);
        flock.updatePosition(100, 100);
        const std::vector<bd::Boid> &flk = flock.getBoids();
        CHECK(flk[0].pos().xcomp() == doctest::Approx(1));
        CHECK(flk[0].pos().ycomp() == doctest::Approx(0.1));
    }
    SUBCASE("testing updatepositions with distant boids (inside the border)")
    {
        bd::Boid b1(Vector(1., 0.), Vector(0., 1.));
        bd::Boid b2(Vector(100., 0.), Vector(1., 1.));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.updatePosition(300, 300);
        const std::vector<bd::Boid> &flk = flock.getBoids();
        CHECK(flk[0].pos().xcomp() == doctest::Approx(1));
        CHECK(flk[0].pos().ycomp() == doctest::Approx(0.1));
        CHECK(flk[1].pos().xcomp() == doctest::Approx(100.1));
        CHECK(flk[1].pos().ycomp() == doctest::Approx(0.1));
    }
    SUBCASE("testing updatepositions with distant boids (outside the border)")
    {
        bd::Boid b1(Vector(1., 0.), Vector(0., 1.));
        bd::Boid b2(Vector(100., 0.), Vector(1., 1.));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.updatePosition(200, 200);
        const std::vector<bd::Boid> &flk = flock.getBoids();
        CHECK(flk[0].pos().xcomp() == doctest::Approx(1));
        CHECK(flk[0].pos().ycomp() == doctest::Approx(0.1));
        CHECK(flk[1].pos().xcomp() == doctest::Approx(-100.0));
        CHECK(flk[1].pos().ycomp() == doctest::Approx(0.1));
    }
    SUBCASE("testing updatepositions with two distant boids and one close boid")
    {
        bd::Boid b1(Vector(1., 0.), Vector(0.0, 1.0));
        bd::Boid b2(Vector(100., 0.), Vector(1.0, 1.0));
        bd::Boid b3(Vector(0.0, 0.0), Vector(1.0, 1.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        flock.updatePosition(300, 300);
        const std::vector<bd::Boid> &flk = flock.getBoids();
        CHECK(flk[0].pos().xcomp() == doctest::Approx(1.09));
        CHECK(flk[0].pos().ycomp() == doctest::Approx(0.1));
        CHECK(flk[1].pos().xcomp() == doctest::Approx(100.1));
        CHECK(flk[1].pos().ycomp() == doctest::Approx(0.1));
        CHECK(flk[2].pos().xcomp() == doctest::Approx(0.01));
        CHECK(flk[2].pos().ycomp() == doctest::Approx(0.1));
    }
    SUBCASE("testing updatepositions with 3 boids")
    {
        bd::Boid b1(Vector(1., 0.), Vector(0., 1.));
        bd::Boid b2(Vector(0.0, 0.0), Vector(1.0, 1.0));
        bd::Boid b3(Vector(1.0, 1.0), Vector(-1.0, -1.0));
        flock.addBoid(b1);
        flock.addBoid(b2);
        flock.addBoid(b3);
        flock.updatePosition(100, 100);
        const std::vector<bd::Boid> &flk = flock.getBoids();
        CHECK(flk[0].pos().xcomp() == doctest::Approx(1.08));
        CHECK(flk[0].pos().ycomp() == doctest::Approx(-0.03));
        CHECK(flk[1].pos().xcomp() == doctest::Approx(-0.135));
        CHECK(flk[1].pos().ycomp() == doctest::Approx(-0.03));
        CHECK(flk[2].pos().xcomp() == doctest::Approx(1.055));
        CHECK(flk[2].pos().ycomp() == doctest::Approx(1.16));
    }
}