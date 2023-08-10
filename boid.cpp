#include "boid.hpp"
#include <random>

double randomNumber()
{
    return -3.0 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 6.0)); // generate a random number between -3. and 3.
}
Boid::Boid(Vector r, Vector v) : position_{r}, velocity_{v} {};
Boid::Boid(Vector r) : position_{r}, velocity_{randomNumber(), randomNumber()} {}; // acceleration is already initialized
Boid::Boid() : position_{0., 0.}, velocity_{randomNumber(), randomNumber()} {}
Boid &Boid::operator=(const Boid &other)
{
    if (this == &other)
    {
        return *this;
    }
    position_ = other.position_;
    velocity_ = other.velocity_;
    acceleration_ = other.acceleration_;
    return *this;
}

double Boid::speed()
{
    return magnitude(velocity_);
}
Vector Boid::setPosition(double x, double y)
{
    position_ = Vector(x, y);
    return position_;
}

Vector Boid::setVelocity(double vx, double vy)
{
    velocity_ = Vector(vx, vy);
    return velocity_;
}
Vector Boid::centerOfMass()
{
    if (boids.empty())
    {
        return Vector(0.0, 0.0);
    }
    int neighborCount = 0; // only close boids count in the formula not all of them
    Vector vSum(0.0, 0.0);
    for (const Boid &b : boids)
    {
        double d = distance(position_, b.pos());
        if (d > 0 && d < ds)
        {
            vSum = vSum + b.pos();
            neighborCount++;
        }
    }
    if (neighborCount > 0)
    {
        vSum = vSum / neighborCount;
    }
    return vSum;
}

Vector Boid::separate()
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
            vSum = vSum + v;
        }
    }
    return -separationFactor * vSum;
}

Vector Boid::cohere()
{
    if (boids.empty())
    {
        return Vector(0.0, 0.0);
    }
    Vector center = centerOfMass();
    if (center == Vector(0.0, 0.0))
    {
        return Vector(0.0, 0.0);
    }
    Vector desired = center - position_;
    return cohesionFactor * desired;
}

Vector Boid::align()
{
    if (boids.empty())
    {
        return Vector(0.0, 0.0);
    }
    Vector vSum(0.0, 0.0);
    int neighborCount = 0;
    for (const Boid &b : boids)
    {
        double d = distance(position_, b.pos());
        if (d > 0 && d < ds)
        {
            Vector v = b.vel() - velocity_;
            vSum = vSum + v;
            neighborCount++;
        }
    }
    if (neighborCount == 0)
    {
        return Vector(0.0, 0.0); // Return zero alignment if there are no neighbors
    }

    Vector averageVelocity = vSum / neighborCount;
    return alignmentFactor * averageVelocity;
}
// update position and border to add
