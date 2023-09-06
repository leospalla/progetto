#ifndef BOID_HPP
#define BOID_HPP
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
  double cohesionFactor{0.8};
  double alignmentFactor{0.5};

  Boid();                   // default constructor
  Boid(int);                // random position constructor for main
  Boid(double, double);     // constructor with position coordinates
  Boid(const vc::Vector&);  // constructor with position vector
  Boid(const vc::Vector&, const vc::Vector&);  // constructor with both vectors

  Boid& operator=(const Boid&);
  bool operator==(const Boid&) const;

  // set functions
  void setPerceptionRadius(double);
  void setSeparationDistance(double);
  void setSeparationFactor(double);
  void setCohesionFactor(double);
  void setAlignmentFactor(double);
  void setPosition(double, double);
  void setVelocity(double, double);

  // get functions
  inline vc::Vector getPosition() const { return m_position; }
  inline vc::Vector getVelocity() const { return m_velocity; }
  inline double getMaxSpeed() const { return m_maxSpeed; }
  inline double getSpeed() const { return m_velocity.Magnitude(); }

  vc::Vector centerOfMass(const std::vector<Boid>&) const;

  // flight rules
  vc::Vector separate(const std::vector<Boid>&) const;
  vc::Vector cohere(const std::vector<Boid>&) const;
  vc::Vector align(const std::vector<Boid>&) const;

  void borders(int);
};
}  // namespace bd
#endif