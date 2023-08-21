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
  vc::Vector m_position{};
  vc::Vector m_velocity{};
  double const m_maxSpeed{10.};
  double m_perceptionRadius{15.};

 public:
  double separationDistance{2.};
  double separationFactor{1.2};
  double cohesionFactor{.8};
  double alignmentFactor{0.5};

  Boid();                // default constructor
  Boid(double, double);  // constructor with position coordinates
  Boid(vc::Vector);          // constructor with position vector
  Boid(vc::Vector, vc::Vector);  // constructor with both vectors

  Boid& operator=(const Boid&);
  bool operator==(const Boid&) const;

  vc::Vector getPosition() const { return m_position; }
  vc::Vector getVelocity() const { return m_velocity; }
  double getMaxSpeed() const { return m_maxSpeed; }
  double getPreceptionRadius() const { return m_perceptionRadius; }

  void setPosition(double, double);
  void setVelocity(double, double);
  void setSeparationDistance(double);
  void setSeparationFactor(double);
  void setCohesionFactor(double);
  void setAlignmentFactor(double);
  double getSpeed() const;

  vc::Vector centerOfMass(std::vector<Boid>) const;
  vc::Vector separate(std::vector<Boid>) const;
  vc::Vector cohere(std::vector<Boid>) const;
  vc::Vector align(std::vector<Boid>) const;

  void borders(unsigned int, unsigned int);
};
}  // namespace bd
#endif