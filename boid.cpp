#include "boid.hpp"
#include <random>

double randomNumber()
{
    return -3.0 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 6.0)); // generate a random number between -3. and 3.
}
Boid::Boid(Vector r, Vector v) : position_{r}, velocity_{v} {};
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
Vector Boid::centerOfMass()
{
    if (boids.empty())
    {
        return Vector(0.0, 0.0);
    }

    Vector vSum(0.0, 0.0);
    for (const Boid &b : boids)
    {
        vSum += b.pos();
    }
    return vSum / boids.size(); // i dont get why it is n-1 in the rules page. for 1 boid is /0.
}

Vector Boid::separate() // should be ok
{
    if (boids.empty())
    {
        return Vector(0.0, 0.0);
    }
    Vector vSum(0.0, 0.0);
    for (const Boid &b : boids)
    {
        double d = distance(position_, b.pos());
        if (d > 0 && d < ds)
        {
            Vector v = b.pos() - position_;
            vSum += v;
        }
    }
    return -separationFactor * vSum;
}

Vector Boid::cohere()// i still have doubts, Leo's may be better
{
    if (boids.empty())
    {
        return Vector(0.0, 0.0);
    }
    Vector center = centerOfMass();
    Vector desired = center - position_;
    return cohesionFactor * desired;
}

Vector Boid::align() // should be ok
{
    if (boids.empty())
    {
        return Vector(0.0, 0.0);
    }
    Vector vSum(0.0, 0.0);
    for (const Boid &b : boids)
    {
        double d = distance(position_, b.pos());
        if (d > 0 && d < ds)
        {
            Vector v = b.vel() - velocity_;
            vSum += v;
        }
    }
    Vector averageVelocity = vSum / boids.size();
    return alignmentFactor * (averageVelocity - velocity_);
}
void Boid::updateVelocity()
{
    Vector separatioVelocity = separate();
    Vector cohesionVelocity = cohere();
    Vector alignmentVelocity = align();
    velocity_ += separatioVelocity + cohesionVelocity + alignmentVelocity;
    if (speed() > maxSpeed_)
    {
        velocity_ = (maxSpeed_ / speed()) * velocity_; //it may be better than creating the function in Vector
    }
}
