#include "boids.hpp"

namespace bd {

// generate a random number between -7. and 7. to not exceed maxSpeed
double randomVelocity() {
  return -7.0 +
         static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 14.0));
}

// between -30 and 30 (used for testing)
double randomPosition() {
  return -30.0 +
         static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 60.));
}

//overload used for input
double randomPosition(int x) {
  return -x +
         static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / x / 2));
}

// constructors
Boid::Boid()
    : m_position{randomPosition(), randomPosition()},
      m_velocity{randomVelocity(), randomVelocity()} {
  int maxAttempts = 100;
  int attempts = 0;
  while (getSpeed() == 0 && attempts < maxAttempts) {
    m_velocity.set(randomVelocity(), randomVelocity());
    ++attempts;
  }
  if (getSpeed() == 0) {
    throw std::runtime_error(
        "Unable to generate non-zero velocity after 100 attempts.");
  }
}

Boid::Boid(int x)
    : m_position{randomPosition(x), randomPosition(x)},
      m_velocity{randomVelocity(), randomVelocity()} {
  int maxAttempts = 100;
  int attempts = 0;
  while (getSpeed() == 0 && attempts < maxAttempts) {
    m_velocity.set(randomVelocity(), randomVelocity());
    ++attempts;
  }
  if (getSpeed() == 0) {
    throw std::runtime_error(
        "Unable to generate non-zero velocity after 100 attempts.");
  }
}

Boid::Boid(double x, double y)
    : m_position{x, y}, m_velocity{randomVelocity(), randomVelocity()} {
  int maxAttempts = 100;
  int attempts = 0;
  while (getSpeed() == 0 && attempts < maxAttempts) {
    m_velocity.set(randomVelocity(), randomVelocity());
    ++attempts;
  }
  if (getSpeed() == 0) {
    throw std::runtime_error(
        "Unable to generate non-zero velocity after 100 attempts.");
  }
}

Boid::Boid(vc::Vector position)
    : m_position{position}, m_velocity{randomVelocity(), randomVelocity()} {
  int maxAttempts = 100;
  int attempts = 0;
  while (getSpeed() == 0 && attempts < maxAttempts) {
    m_velocity.set(randomVelocity(), randomVelocity());
    ++attempts;
  }
  if (getSpeed() == 0) {
    throw std::runtime_error(
        "Unable to generate non-zero velocity after 100 attempts.");
  }
}

Boid::Boid(vc::Vector position, vc::Vector velocity)
    : m_position{position}, m_velocity{velocity} {
      m_velocity.limit(m_maxSpeed);
    }

// set functions
void Boid::setPosition(double x, double y) { m_position.set(x, y); }

void Boid::setVelocity(double vx, double vy) {
  m_velocity.set(vx, vy);
  m_velocity.limit(m_maxSpeed);
}

void Boid::setPerceptionRadius(double x) { perceptionRadius = x; }
void Boid::setSeparationDistance(double x) { separationDistance = x; }
void Boid::setSeparationFactor(double x) { separationFactor = x; }
void Boid::setCohesionFactor(double x) { cohesionFactor = x; }
void Boid::setAlignmentFactor(double x) { alignmentFactor = x; }

// operator overloading
Boid& Boid::operator=(const Boid& other) {
  if (this == &other) {
    return *this;
  }
  m_position = other.m_position;
  m_velocity = other.m_velocity;
  return *this;
}

bool Boid::operator==(const Boid& other) const {
  return m_position == other.m_position && m_velocity == other.m_velocity;
}

// flight rules
vc::Vector Boid::separate(std::vector<Boid> boids) const {
  if (boids.empty()) {
    return vc::Vector{0.0, 0.0};
  }
  vc::Vector vSum{0.0, 0.0};
  for (const Boid& other : boids) {
    double distance = m_position.distance(other.getPosition());
    if (distance > 0 && distance < separationDistance) {
      vc::Vector direction = other.getPosition() - m_position;
      vSum += direction;
    }
  }
  vc::Vector separation = vSum * -separationFactor;
  separation.limit(m_maxSpeed);
  return separation;
}

vc::Vector Boid::cohere(std::vector<Boid> boids) const {
  if (boids.empty()) {
    return vc::Vector{0., 0.};
  }
  vc::Vector currentPosition = m_position;
  vc::Vector centerOfMass{0.0, 0.0};
  int neighborCount = 0;

  for (const Boid& otherBoid : boids) {
    double distance = currentPosition.distance(otherBoid.getPosition());
    if (distance > 0 && distance < perceptionRadius) {
      centerOfMass += otherBoid.getPosition();
      ++neighborCount;
    }
  }
  if (neighborCount > 0) {
    centerOfMass = centerOfMass / static_cast<double>(neighborCount);
    vc::Vector cohesion = (centerOfMass - currentPosition) * cohesionFactor;
    cohesion.limit(m_maxSpeed);
    return cohesion;
  } else {
    return vc::Vector{0., 0.};
  }
}

vc::Vector Boid::align(std::vector<Boid> boids) const {
  if (boids.empty()) {
    return vc::Vector{0.0, 0.0};
  }

  vc::Vector currentPosition = m_position;
  vc::Vector averageVelocity{0.0, 0.0};
  int neighborCount = 0;

  for (const Boid& otherBoid : boids) {
    double distance = currentPosition.distance(otherBoid.getPosition());
    if (distance > 0 && distance < perceptionRadius) {
      averageVelocity += otherBoid.getVelocity();
      ++neighborCount;
    }
  }
  if (neighborCount > 0) {
    averageVelocity = averageVelocity / static_cast<double>(neighborCount);
    vc::Vector alignment = (averageVelocity - m_velocity) * alignmentFactor;
    alignment.limit(m_maxSpeed);
    return alignment;
  } else {
    return vc::Vector{0.0, 0.0};
  }
}

vc::Vector Boid::centerOfMass(std::vector<Boid> boids) const {
  if (boids.empty()) {
    return vc::Vector{0.0, 0.0};
  }
  int neighborCount = 0;
  vc::Vector vSum{0.0, 0.0};
  for (const Boid& other : boids) {
    double distance = m_position.distance(other.getPosition());
    if (distance > 0 && distance < perceptionRadius) {
      vSum = vSum + other.getPosition();
      ++neighborCount;
    }
  }
  if (neighborCount > 0) {
    vSum = vSum / neighborCount;
  } else {
    return m_position;
  }
  return vSum;
}

void Boid::borders(int length) {
  double halfWidth = length / 2;
  double halfHeight = length / 2;
  if (m_position.getX() > halfWidth) {
    setPosition(-halfWidth, m_position.getY());
  }
  if (m_position.getX() < -halfWidth) {
    setPosition(halfWidth, m_position.getY());
  }
  if (m_position.getY() > halfHeight) {
    setPosition(m_position.getX(), -halfHeight);
  }
  if (m_position.getY() < -halfHeight) {
    setPosition(m_position.getX(), halfHeight);
  }
}
}  // namespace bd
