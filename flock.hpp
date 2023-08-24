#ifndef FLOCK_HPP
#define FLOCK_HPP
#include <algorithm>
#include <iostream>

#include "boid.hpp"
namespace fk
{
  class Flock
  {
  private:
    std::vector<bd::Boid> m_boids;
    double const m_deltaTime{.1};

  public:
    Flock();

    const std::vector<bd::Boid> &getBoids() const { return m_boids; }
    double getDeltaTime() const { return m_deltaTime; }

    void addBoid(const bd::Boid &boid);
    void removeBoid(const bd::Boid &boid);

    void updateVelocity();
    void updatePosition(unsigned int, unsigned int);
    void updateBoidParameters(double, double, double, double, double); // needs testing

    double averageDistance() const;
    double averageSpeed() const;

    double standardDeviationDistance() const;
    double standardDeviationSpeed() const;
    int countFlocks() const;
    std::vector<int> countBoidsInFlock();
    // std::vector<int> reOrderBoidsInFlock(std::vector<int>, std::vector<bd::Boid>);

    void simulate(int, unsigned int, unsigned int);
  };
} // namespace fk
#endif