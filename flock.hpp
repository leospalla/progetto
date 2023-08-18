#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "boid.hpp"
#include <algorithm>
#include <iostream>
class Flock
{
private:
    std::vector<Boid> m_boids;
    double const delta_time{0.1};

public:
    Flock();
    const std::vector<Boid>& getBoids() const { return m_boids; }
    double getDeltaTime() const { return delta_time;}
    void addBoid(const Boid &boid);
    void removeBoid(const Boid &boid);
    void updateVelocity();
    void updatePosition(unsigned int, unsigned int);
    double averageDistance();
    double averageSpeed();
    double standardDeviationDistance();
    double standardDeviationSpeed();
    void simulate(int, unsigned int, unsigned int);
};
#endif