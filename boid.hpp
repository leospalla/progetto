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
    Boid(Vector);
    Boid();
    Vector pos() { return position_; }
    Vector vel() { return velocity_; }
    Vector acc() { return acceleration_; }
    Vector setPosition(double, double);
    Vector setVelocity(double, double);
    double speed();
};
double const ds{1.}; // max distance for separation, probably should go private
double separationfactor{1.}; // i have initialized this variables arbitrarly, this can and should change.
double cohesionfactor{1.};
double allignmentfactor{1.};
std::vector<Boid> boids;
Vector separate(); //it may be possible to put these in the private idk
Vector cohere();
Vector align();
Vector centerOfMass();
void update();
#endif