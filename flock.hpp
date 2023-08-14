#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "boid.hpp"
#include <algorithm>
#include <iostream>
class Flock
{
private:
    std::vector<Boid> m_boids;
    double const delta_time{1.};

public:
    Flock();
    const std::vector<Boid>& getBoids() const { return m_boids; }
    void addBoid(const Boid &boid);
    void removeBoid(const Boid &boid);
    void updateVelocity();
    void updatePosition();
    double averageDistance();
    double averageSpeed();
    double standardDeviationDistance();
    double standardDeviationSpeed();
    void simulate(int);
};
#endif