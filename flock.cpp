#include "flock.hpp"

Flock::Flock() : m_boids{std::vector<Boid>()} {}; // constructor, empty container of Boids.
void Flock::addBoid(const Boid& boid)
{
    m_boids.push_back(boid);
}
void Flock::updateVelocity()
{
    for (Boid &b : m_boids)
    {
        Vector separatioVelocity = b.separate();
        Vector cohesionVelocity = b.cohere();
        Vector alignmentVelocity = b.align();
        Vector totalVelocity = separatioVelocity + cohesionVelocity + alignmentVelocity;
        b.setVelocity(b.vel().xcomp() + totalVelocity.xcomp(), b.vel().xcomp() + totalVelocity.ycomp());
        if (b.speed() > b.maxSpeed())
        {
            b.vel() = (b.maxSpeed() / b.speed()) * b.vel(); // it may be better than creating the function in Vector
        }
    }
}
void Flock::updatePosition()
{
    for (Boid &b : m_boids)
    {
        Vector vSum = delta_time * b.vel();
        b.setPosition(b.pos().xcomp() + vSum.xcomp(), b.pos().ycomp() + vSum.ycomp());
    }
}