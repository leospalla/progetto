#ifndef FLOCK_HPP
#define FLOCK_HPP
#include <algorithm>
#include <iostream>
#include <stack>
#include "boids.hpp"
namespace fk {
class Flock {
 private:
  std::vector<bd::Boid> m_boids;
  double m_delta_time{.1};

 public:
  Flock();

  const std::vector<bd::Boid> &getBoids() const { return m_boids; }
  double getDeltaTime() const { return m_delta_time; } //no need to set

  void addBoid(const bd::Boid &boid);
  void removeBoid(const bd::Boid &boid); //i dont think we even used this

  void updateVelocity();
  void updatePosition(unsigned int, unsigned int);
  void updateBoidParameters(double, double, double, double, double);

  double averageDistance() const;
  double averageSpeed() const;

  double standardDeviationDistance() const;
  double standardDeviationSpeed() const;

  std::vector<int> countBoidsInFlock() const;

  void simulate(int, unsigned int, unsigned int);
};
}  // namespace fk
#endif