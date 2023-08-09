#ifndef BOID_HPP
#define BOID_HPP
#include "vector.hpp"
#include <vector>
class Boid
{
private:
    Vector position_{};
    Vector velocity_{};
    Vector acceleration_{0., 0.};
    double const maxSpeed_{5.};

public:
    Boid &operator=(const Boid &other);//it is needed to assign the values of a boid to another boid. I dint use it but it may be usefull.
    double const ds{5.};
    double const separationFactor{1.};
    double const cohesionFactor{1.};
    double const alignmentFactor{1.};
    Boid(Vector, Vector); // Its needed for tests
    Boid(Vector);
    Boid();
    Vector pos() const { return position_; }
    Vector vel() const { return velocity_; }
    Vector acc() const { return acceleration_; }
    double maxSpeed() const {return maxSpeed_;}
    Vector setPosition(double, double);
    Vector setVelocity(double, double);
    Vector centerOfMass();
    Vector separate(); // I left them in the public because it's easier to test.
    Vector cohere();
    Vector align();
    double speed();
    // border to add
    std::vector<Boid> boids;
};
#endif