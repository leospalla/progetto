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
    double const ds{5.};
    double const separationFactor{1.};
    double const cohesionFactor{1.};
    double const alignmentFactor{1.};
    Boid(Vector, Vector); // its needed for tests
    Boid(Vector);
    Boid();
    Vector pos() const { return position_; }
    Vector vel() const { return velocity_; }
    Vector acc() const { return acceleration_; }
    Vector setPosition(double, double);
    Vector setVelocity(double, double);
    Vector centerOfMass();
    Vector separate(); //i left them in the public because i dont know if it works in the private
    Vector cohere();
    Vector align();
    double speed();
    void updateVelocity(); //adds the velocity of the rules
    std::vector<Boid> boids;
};
#endif