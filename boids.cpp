#include "boids.hpp"


// global variable to memorize perceptionRadius taken as input
double g_perceptionRadius = 10.;

// generate a random number between -3. and 3.
double randomNumber() {
  return -3.0 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 6.0));
}

// constructors
Boid::Boid()
    : m_position{0., 0.},  // should be created in the same position or random?
      m_velocity{randomNumber(), randomNumber()},
      m_perceptionRadius{g_perceptionRadius} {
  int maxAttempts = 100;
  int attempts = 0;
  while (getSpeed() == 0 && attempts < maxAttempts) {
    m_velocity.x = randomNumber();
    m_velocity.y = randomNumber();
    attempts++;
  }
  if (getSpeed() == 0) {
    throw std::runtime_error("Unable to generate non-zero velocity after 100 attempts.");
  }

//  boids.push_back(*this);
}

Boid::Boid(double x, double y)
    : m_position{x, y},
      m_velocity{randomNumber(), randomNumber()},  // random generated but it must be 0 < |velocity| < maxspeed,
      m_perceptionRadius{g_perceptionRadius} {
  int maxAttempts = 100;
  int attempts = 0;
  while (getSpeed() == 0 && attempts < maxAttempts) {
    m_velocity.x = randomNumber();
    m_velocity.y = randomNumber();
    attempts++;
  }
  if (getSpeed() == 0) {
    throw std::runtime_error("Unable to generate non-zero velocity after 100 attempts.");
  }
//  boids.push_back(*this);

}

Boid::Boid(Vector position)
    : m_position{position},
      m_velocity{randomNumber(), randomNumber()},
      m_perceptionRadius{g_perceptionRadius} {
  int maxAttempts = 100;
  int attempts = 0;
  while (getSpeed() == 0 && attempts < maxAttempts) {
    m_velocity.x = randomNumber();
    m_velocity.y = randomNumber();
    attempts++;
  }
  if (getSpeed() == 0) {
    throw std::runtime_error("Unable to generate non-zero velocity after 100 attempts.");
  }
//  boids.push_back(*this);
}

Boid::Boid(Vector position, Vector velocity)
    : m_position{position},
      m_velocity{velocity},
      m_perceptionRadius{g_perceptionRadius} {
//  boids.push_back(*this);
}
/*
In the main.cpp we will use something like
std::cout << "Insert perceptionRadius: " ;
std::cin >> g_perceptionRadius;

so in this way we can create every boid with the same perceptionRadius
without using a default value, but taking its value as input as requested.
*/

void Boid::setPosition(double x, double y) { 
  m_position.x = x;
  m_position.y = y;
 }

void Boid::setVelocity(double vx, double vy) { 
  m_velocity.x = vx;
  m_velocity.y = vy;
  m_velocity.limit(m_maxSpeed);
 }

// we need to do a double getSpeed() and assert that speed must be more than 0.
// and less than maxSpeed without changing the angle

double Boid::getSpeed() { return m_velocity.Magnitude(); }

Vector Boid::separate(std::vector<Boid> boids) {
  if (boids.empty()) {
    return Vector{0., 0.};
  }
  Vector currentPosition = m_position;
  auto it = boids.begin();
  auto const last = boids.end();
  Vector vSum(0.0, 0.0);
  while (it != last) {  // while loop that uses iterators to repeat the
                        // operation for each element of boids
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

Vector Boid::cohere(std::vector<Boid> boids) {
  if (boids.empty()) {
    return Vector{0., 0.};
  }
  Vector currentPosition = m_position;
  Vector centerOfMass(0.0, 0.0);
  int neighborCount = 0;

  for (const Boid& otherBoid : boids) {  // for-each loop
    double distance = currentPosition.distance(otherBoid.getPosition());
    if (distance > 0 && distance < m_perceptionRadius) {
      centerOfMass += otherBoid.getPosition();
      neighborCount++;
    }
  }
  if (neighborCount > 0) {
    centerOfMass = centerOfMass /static_cast<double>(neighborCount);
    Vector cohesion = centerOfMass - currentPosition;
    cohesion = cohesion.Normalize() * m_maxSpeed - m_velocity;
    cohesion = cohesion * cohesionFactor;
    cohesion.limit(m_maxSpeed);
    return cohesion;
  } else {
    return Vector{0., 0.};
  }
}

Vector Boid::align(std::vector<Boid> boids) {
  if (boids.empty()) {
    return Vector(0.0, 0.0);
  }

  Vector currentPosition = m_position;
  Vector averageVelocity(0.0, 0.0);
  int neighborCount = 0;

  for (const Boid& otherBoid : boids) {
    double distance = currentPosition.distance(otherBoid.getPosition());
    if (distance > 0 && distance < m_perceptionRadius) {
      averageVelocity += otherBoid.getVelocity();
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

// updates the position, the velocity, and the acceleration of each boid
void Boid::update(std::vector<Boid> boids) {  
  m_acceleration = align(boids); //+ separate(boids) + cohere(boids);
  m_velocity += m_acceleration;
  m_velocity.limit(m_maxSpeed);
  m_position += m_velocity;
  m_acceleration.Set(0., 0.);
}


/*
#include "SFML/Graphics.hpp"

// variable for borders()
sf::VideoMode desktopTemp = sf::VideoMode::getDesktopMode();
const int window_height = desktopTemp.height;
const int window_width = desktopTemp.width;
#define w_height window_height
#define w_width window_width

void Boid::borders() {
  if (m_position.x < 0) {
    m_position.x = w_width;
  } else if (m_position.x > w_width) {
    m_position.x = 0;
  }
  if (m_position.y < 0) {
    m_position.y = w_height;
  } else if (m_position.y > w_height) {
    m_position.y = 0;
  }
}

*/

