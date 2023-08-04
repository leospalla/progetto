#include "boid.hpp"
#include <random>

double randomNumber()
{
    return -3.0 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 6.0)); // generate a random number between -3. and 3.
}
Boid::Boid(Vector r) : position_{r}, velocity_{randomNumber(), randomNumber()} {}; // acceleration is already initialized
Boid::Boid() : position_{0., 0.}, velocity_{randomNumber(), randomNumber()} {}
double Boid::speed()
{
    return magnitude(velocity_);
}
Vector Boid::setPosition(double x, double y)
{
    return Vector(x, y);
}
Vector Boid::setVelocity(double vx, double vy)
{
    return Vector(vx, vy);
}
Vector centerOfMass()
{
    auto it = boids.begin();
    auto const last = boids.end();
    Vector v(0., 0.);
    while (it != last)
    {
        Boid b = *it;
        v += b.pos();
        ++it;
    }
    return (1 / (boids.size() - 1)) * v;
}
Vector separate()
{
    auto it = boids.begin();
    auto const last = boids.end();
    Boid b1 = boids[0];
    Vector v1(0., 0.);
    while (it != last)
    {
        Boid b2 = *it;
        double d = distance(b1.pos(), b2.pos());
        if (d > 0 && d < ds)
        {
            Vector v = b2.pos() - b1.pos();
            v1 += v;
        }
        ++it;
    }
    return -separationfactor * v1;
}
Vector cohere()
{
    Boid b;
    return cohesionfactor * (centerOfMass() - b.pos()); //this probably makes no sense
}
Vector align()
{
   auto it = boids.begin();
    auto const last = boids.end();
    Boid b1 = boids[0];
    Vector v1(0., 0.);
    while (it != last)
    {
        Boid b2 = *it;
        Vector v = b2.vel() - b1.vel();
        v1 += v;  
        ++it;
    }
    Vector v2 = (1 / (boids.size() - 1)) * v1;
    return cohesionfactor * v2; 
}
//the correction of the velocity is missing

/* 
This is a pritive version. It is very likley to find mistakes.
In the rules maybe we could use some algorithms.
The programm is succesfully compiled with g++ but i didnt do any tests so idk if it works.
*/