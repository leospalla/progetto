#include "boids.hpp"
#include <random>

double randomNumber() {  // generate a random number between -3. and 3.
  return -3.0 +
         static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 6.0));
}

double g_perceptionRadius = 0. ; //global variable to memorize perceptionRadius taken as input

// constructors
Boid::Boid()
    : m_position{0., 0.},  // should be created in the same position or random?
      m_velocity{randomNumber(), randomNumber()},
      //maxSpeed and acceleration are already initialized in the private
      m_perceptionRadius{g_perceptionRadius} {}

Boid::Boid(double x, double y)

    : m_position{x, y},
      m_velocity{randomNumber(), randomNumber()},  // random generated but it must be 0. <|velocity| < maxspeed,
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
