#include "boids.hpp"
#include <random>
#include <ctime>
#include <cstdlib>

double randomNumber() { //generate a random number between -3. and 3.
return -3.0 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 6.0));
}

// constructors
Boid::Boid()
    : m_position{0., 0.}, //should be created in the same position or random?
      m_velocity{randomNumber(),randomNumber()},
      m_maxSpeed{m_maxSpeed},
      m_perceptionRadius{perceptionRadius} {};

Boid::Boid(double x, double y) 
    : m_position{x, y},
      m_velocity{randomNumber(),randomNumber()},  // random generated but it must be 0. < |velocity| < maxspeed
      m_maxSpeed{m_maxSpeed},
      m_perceptionRadius{perceptionRadius} {}  

Boid::Boid(Vector position)
    : m_position{position},
      m_velocity{randomNumber(),randomNumber()},
      m_maxSpeed{m_maxSpeed},
      m_perceptionRadius{perceptionRadius} {};

//we need to do a double getSpeed() and assert that speed must be more than 0. and less than maxSpeed

