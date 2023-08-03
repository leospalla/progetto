#include "boids.hpp"

#include <random>

double randomNumber() {  // generate a random number between -3. and 3.
  return -3.0 +
         static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 6.0));
}

double g_perceptionRadius =
    0.;  // global variable to memorize perceptionRadius taken as input

// constructors
Boid::Boid()
    : m_position{0., 0.},  // should be created in the same position or random?
      m_velocity{randomNumber(), randomNumber()},
      // maxSpeed and acceleration are already initialized in the private
      m_perceptionRadius{g_perceptionRadius} {}

Boid::Boid(double x, double y)

    : m_position{x, y},
      m_velocity{randomNumber(),
                 randomNumber()},  // random generated but it must be 0.
                                   // <|velocity| < maxspeed,
      m_perceptionRadius{g_perceptionRadius} {}

Boid::Boid(Vector position)
    : m_position{position},
      m_velocity{randomNumber(), randomNumber()},
      m_perceptionRadius{g_perceptionRadius} {}

/*
In the main.cpp we will use something like
std::cout << "Insert perceptionRadius: " ;
std::cin >> g_perceptionRadius;

so in this way we can create every boid with the same perceptionRadius
without using a default value, but taking its value as input as its
requested.
*/

Vector Boid::setPosition(double x, double y) { return Vector(x, y); }

Vector Boid::setVelocity(double vx, double vy) { return Vector(vx, vy); }

// we need to do a double getSpeed() and assert that speed must be more than 0.
// and less than maxSpeed without changing the angle

double Boid::getSpeed() { return m_velocity.Magnitude(); }

Vector Boid::separate() {
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
// and it should be the sum of separation, cohesion and alignment

Vector Boid::cohere() {
  if (boids.empty()) {
    return Vector{0., 0.};
  }
  Vector currentPosition = m_position;
  Vector centerOfMass(0.0, 0.0);
  int neighborCount = 0;

  for (const Boid& otherBoid : boids) {  // for-each loop
    double distance = currentPosition.distance(otherBoid.getPosition());
    if (distance > 0 && distance < perceptionRadius) {
      centerOfMass += otherBoid.getPosition();
      neighborCount++;
    }
  }
  if (neighborCount > 0) {
    centerOfMass = centerOfMass /static_cast<double>(neighborCount);  // static_cast because neighborCount is an int
    Vector cohesion = centerOfMass - currentPosition;
    cohesion = cohesion.Normalize() * m_maxSpeed - m_velocity;
    cohesion.limit(m_maxSpeed);
    return cohesion;
  } else {
    return Vector{0., 0.};
  }
}

Vector Boid::align() {
  if (boids.empty()) {
    return Vector(0.0, 0.0);
  }

  Vector currentPosition = m_position;
  Vector averageVelocity(0.0, 0.0);
  int neighborCount = 0;

  for (const Boid& otherBoid : boids) {  // for-each loop that iterates for each element of boids
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
