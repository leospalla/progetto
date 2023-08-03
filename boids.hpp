#ifndef BOIDS_HPP
#define BOIDS_HPP
#include "vector.hpp"
#include <vector>



class Boid {
 private:
  Vector m_position{};
  Vector m_velocity{};
  Vector m_acceleration{};
  double const m_maxSpeed{5.};  // here we should put a default
  double m_perceptionRadius;    // this needs to be taken as input

  // should we add a class invariant? (it needs to go in the public anyway)

  Vector separate();  // we need to create a steer vector that is added to the
                      // velocity of the vector of the boid
  Vector cohere();
  Vector align();

 public:
  double const perceptionRadius{};
  double separationDistance;  // must be less than the perception radius
  double separationFactor;
  double cohesionFactor;
  double alignmentFactor;  // must be less than 1

  Boid();                  // default constructor
  Boid(double, double);    // constructor with position coordinates
  Boid(Vector);            // constructor with position vector

  Vector getPosition() { return m_position; }
  Vector getVelocity() { return m_velocity; }
  Vector setPosition(double, double);
  Vector setVelocity(double, double);

  double getSpeed();
  void update();  // for now i put void but i dont know if its right, update
                  // means it should update the velocity of each boid
};

#endif
