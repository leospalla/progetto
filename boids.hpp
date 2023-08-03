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

  Vector separate() {
    if (boids.empty()) {
      return Vector{0., 0.};
    }
    Vector currentPosition = m_position;
    auto it = boids.begin();
    auto const last = boids.end();
    Vector vSum(0.0, 0.0);
    while (it != last) {  // while loop that uses iterators to repeat the operation for each element of boids
      const Vector& otherPosition = it->getPosition();
      double distance = currentPosition.distance(otherPosition);
      if (distance > 0 && distance < separationDistance) {
        Vector direction = otherPosition - currentPosition;
        vSum += direction;
      }
      ++it;
    }
    Vector separation = vSum * -separationFactor;
    separation.limit(m_maxSpeed);
    return separation;
  }

  // we need to create a steer vector that is added to the
  // velocity of the vector of the boid (steer = acceleration, its the same)
  //and it should be the sum of separation, cohesion and alignment

  Vector cohere() {
    if (boids.empty()) {
      return Vector{0., 0.};
    }
    Vector currentPosition = m_position; 
    Vector centerOfMass(0.0, 0.0);  
    int neighborCount = 0;

    for (const Boid& otherBoid : boids) { //for-each loop
      double distance = currentPosition.distance(otherBoid.getPosition());
      if (distance > 0 && distance < perceptionRadius) {
        centerOfMass += otherBoid.getPosition();
        neighborCount++;
      }
    }
    if (neighborCount > 0) {
      centerOfMass = centerOfMass / static_cast<double>(neighborCount); //static_cast because neighborCount is an int
      Vector cohesion = centerOfMass - currentPosition;
      cohesion = cohesion.Normalize() * m_maxSpeed - m_velocity;
      cohesion.limit(m_maxSpeed);
      return cohesion;
    } else {
      return Vector{0., 0.};
    }
  }

Vector align() {
  if (boids.empty()) {
    return Vector(0.0, 0.0);
  }

  Vector currentPosition = m_position;
  Vector averageVelocity(0.0, 0.0);
  int neighborCount = 0;

  for (const Boid& otherBoid :
       boids) {  // for-each loop that iterates for each element of boids
    double distance = currentPosition.distance(otherBoid.getPosition());
    if (distance > 0 && distance < perceptionRadius) {
      averageVelocity = averageVelocity + otherBoid.getVelocity();
      neighborCount++;
    }
  }
  if (neighborCount > 0) {
    averageVelocity = averageVelocity / static_cast<double>(neighborCount);
    Vector alignment = averageVelocity.Normalize() * m_maxSpeed - m_velocity;
    alignment = alignment * alignmentFactor;
    alignment.limit(m_maxSpeed);
    return alignment; 
  } else {
    return Vector{0.0, 0.0};
  }
}

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
