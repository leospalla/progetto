#ifndef FLOCK_HPP
#define FLOCK_HPP
#include <algorithm>
#include <iostream>

#include "boids.hpp"
namespace fk {
class Flock {
 private:
  std::vector<bd::Boid> m_boids;
  double const delta_time{.1};

 public:
  Flock();

  const std::vector<bd::Boid> &getBoids() const { return m_boids; }
  double getDeltaTime() const { return delta_time; }
  void addBoid(const bd::Boid &boid);
  void removeBoid(const bd::Boid &boid);

  void updateVelocity();
  void updatePosition(unsigned int, unsigned int);

  double averageDistance() const;
  double averageSpeed() const;

  double standardDeviationDistance() const;
  double standardDeviationSpeed() const;

  void simulate(int, unsigned int, unsigned int);
};
}  // namespace fk
#endif