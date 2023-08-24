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
  double const m_delta_time{.1};

 public:
  Flock();

  const std::vector<bd::Boid> &getBoids() const { return m_boids; }
  double getDeltaTime() const { return m_delta_time; }

  void addBoid(const bd::Boid &boid);
  void removeBoid(const bd::Boid &boid);

  void updateVelocity();
  void updatePosition(int);
  void updateBoidParameters(double, double, double, double, double);

  double averageDistance() const;
  double averageSpeed() const;

  double standardDeviationDistance() const;
  double standardDeviationSpeed() const;

  std::vector<int> countBoidsInFlock(double) const;

  void simulate(int, int, double);
};
}  // namespace fk
#endif