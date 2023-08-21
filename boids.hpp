#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

#include "vector.hpp"

namespace bd {
class Boid {
 private:
  Vector m_position{};
  Vector m_velocity{};
  Vector m_acceleration{0., 0.};
  double const m_maxSpeed{10.};
  double m_perceptionRadius{15.};  // this needs to be taken as input or maybe
                                   // not and just used as default

 public:
  double const separationDistance{2.};
  double const separationFactor{1.2};
  double const cohesionFactor{.8};
  double const alignmentFactor{0.5};

  Boid();                // default constructor
  Boid(double, double);  // constructor with position coordinates
  Boid(Vector);          // constructor with position vector
  Boid(Vector, Vector);  // constructor with both vectors

  Boid& operator=(const Boid&);
  bool operator==(const Boid&) const;

  Vector getPosition() const { return m_position; }
  Vector getVelocity() const { return m_velocity; }
  Vector getAcceleration() const { return m_acceleration; }
  double getMaxSpeed() const { return m_maxSpeed; }

  void setPosition(double, double);
  void setVelocity(double, double);
  double getSpeed() const;

  Vector separate(std::vector<Boid>) const;
  Vector cohere(std::vector<Boid>) const;
  Vector align(std::vector<Boid>) const;

  void borders(unsigned int, unsigned int);
};
}  // namespace bd
#endif