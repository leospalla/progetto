#ifndef BOID_HPP
#define BOID_HPP
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
  double const m_maxSpeed{10.};  // class invariant

 public:
  // parameters are already initialized for testing
  double perceptionRadius{15.};
  double separationDistance{2.};
  double separationFactor{1.2};
  double cohesionFactor{.8};
  double alignmentFactor{0.5};

  Boid();                        // default constructor
  Boid(int);                     // random position constructor for main
  Boid(double, double);          // constructor with position coordinates
  Boid(vc::Vector);              // constructor with position vector
  Boid(vc::Vector, vc::Vector);  // constructor with both vectors

  Boid& operator=(const Boid&);
  bool operator==(const Boid&) const;

  void setPerceptionRadius(double);
  void setSeparationDistance(double);
  void setSeparationFactor(double);
  void setCohesionFactor(double);
  void setAlignmentFactor(double);
  void setPosition(double, double);
  void setVelocity(double, double);

  vc::Vector getPosition() const { return m_position; }
  vc::Vector getVelocity() const { return m_velocity; }
  double getMaxSpeed() const { return m_maxSpeed; }
  double getSpeed() const { return m_velocity.Magnitude(); }

  vc::Vector centerOfMass(std::vector<Boid>) const;

  vc::Vector separate(std::vector<Boid>) const;
  vc::Vector cohere(std::vector<Boid>) const;
  vc::Vector align(std::vector<Boid>) const;

  void borders(int);
};
}  // namespace bd
#endif