#ifndef FLOCK_HPP
#define FLOCK_HPP
#include <algorithm>
#include <iostream>

#include "boids.hpp"
namespace fk {
class Flock {
 private:
  std::vector<bd::Boid> m_boids;
  double delta_time{.1};

 public:
  Flock();

  const std::vector<bd::Boid> &getBoids() const { return m_boids; }
  double getDeltaTime() const { return delta_time; } //do we need this? (maybe only if we use it as input in the main)

  void setDeltaTime(double); // do we need this? if yes then also add tests
  void addBoid(const bd::Boid &boid);
  void removeBoid(const bd::Boid &boid); //i dont think we even used this

  void updateVelocity();
  void updatePosition(unsigned int, unsigned int);
  void updateBoidParameters(double, double, double, double, double); //needs testing

  int countFlocks() const;
  double averageDistance() const;
  double averageSpeed() const;

  double standardDeviationDistance() const;
  double standardDeviationSpeed() const;

  void simulate(int, unsigned int, unsigned int);
};
}  // namespace fk
#endif