#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "boidtelly.hpp"
#include <algorithm>
#include <iostream>
class Flock
{
private:
    std::vector<Boid> m_boids;
    double const delta_time{1.};

public:
    Flock();
    void addBoid(const Boid &boid);
    void removeBoid(const Boid &boid);
    void updateVelocity();
    void updatePosition();
    double averageDistance() const;
    double averageSpeed();
    double standardDeviationDistance();
    double standardDeviationSpeed();
    void simulate(int);
    int getnumboids();
    std::vector<Boid> getmboids();
};
#endif