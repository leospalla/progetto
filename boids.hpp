#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <vector>

#include "vector.hpp"

class Boid {
 private:
  Vector m_position{};
  Vector m_velocity{};
  Vector m_acceleration{0., 0.};
  double const m_maxSpeed{5.};  // here we should put a default
  double m_perceptionRadius;    // this needs to be taken as input

  // should we add a class invariant? (it needs to go in the public anyway)

  Vector separate();

  Vector cohere();

  Vector align();

  // we need to create a steer vector that is added to the
  // velocity of the vector of the boid (steer = acceleration, its the same)
  //and it should be the sum of separation, cohesion and alignment

public:
double const perceptionRadius{};
double const separationDistance{};  // must be less than the perception radius
double const separationFactor{};
double const cohesionFactor{};
double const alignmentFactor{};  // must be less than 1

std::vector<Boid> boids;

Boid();                // default constructor
Boid(double, double);  // constructor with position coordinates
Boid(Vector);          // constructor with position vector

Vector getPosition() const { return m_position; }
Vector getVelocity() const { return m_velocity; }
Vector getAcceleration() { return m_acceleration; }
Vector setPosition(double, double);
Vector setVelocity(double, double);

double getSpeed();
void update();  // for now i put void but i dont know if its right, update
                // means it should update the velocity of each boid
};

#endif
