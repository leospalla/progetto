#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <vector>
#include <random>
#include <stdexcept>
#include <iostream>

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



public:
double const separationDistance{1.};  // must be less than the perception radius
double const separationFactor{1.};
double const cohesionFactor{1.};
double const alignmentFactor{0.5};  // must be less than 1

std::vector<Boid> boids;

Boid();                // default constructor
Boid(double, double);  // constructor with position coordinates
Boid(Vector);          // constructor with position vector

Vector getPosition() const { return m_position; }
Vector getVelocity() const { return m_velocity; }
Vector getAcceleration() const { return m_acceleration; }

void setPosition(double, double);
void setVelocity(double, double);

double getSpeed();
  Vector align();
void update(); 
void borders();
};

#endif
