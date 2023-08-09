#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "boid.hpp"
class Flock
{
    private:
    std::vector<Boid> m_boids;
    double const delta_time{1.};
    public:
    Flock();
    void addBoid(const Boid& boid);
    void updateVelocity();
    void updatePosition();
};
#endif