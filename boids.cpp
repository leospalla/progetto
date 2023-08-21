#include "boids.hpp"

namespace bd {

// generate a random number between -7. and 7.
double randomVelocity() {
  return -7.0 +
         static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 14.0));
}

// between -10 and 10
double randomPosition() {
  return -10.0 +
         static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 20.));
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

Boid::Boid(Vector position)
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

Boid::Boid(Vector position, Vector velocity)
    : m_position{position}, m_velocity{velocity} {}

void Boid::setPosition(double x, double y) { m_position.set(x, y); }

void Boid::setVelocity(double vx, double vy) {
  m_velocity.set(vx, vy);
  m_velocity.limit(m_maxSpeed);
}

Boid& Boid::operator=(const Boid& other) {
  if (this == &other) {
    return *this;
  }
  m_position = other.m_position;
  m_velocity = other.m_velocity;
  m_acceleration = other.m_acceleration;
  return *this;
}

bool Boid::operator==(const Boid& other) const {
  return m_position == other.m_position && m_velocity == other.m_velocity;
}

double Boid::getSpeed() const { return m_velocity.Magnitude(); }

Vector Boid::separate(std::vector<Boid> boids) const {
  if (boids.empty()) {
    return Vector(0.0, 0.0);
  }
  Vector vSum(0.0, 0.0);
  for (const Boid& other : boids) {
    double distance = m_position.distance(other.getPosition());
    if (distance > 0 && distance < separationDistance) {
      Vector direction = other.getPosition() - m_position;
      vSum += direction;
    }
  }
  Vector separation = vSum * -separationFactor;
  separation.limit(m_maxSpeed);
  return separation;
}

Vector Boid::cohere(std::vector<Boid> boids) const {
  if (boids.empty()) {
    return Vector{0., 0.};
  }
  Vector currentPosition = m_position;
  Vector centerOfMass(0.0, 0.0);
  int neighborCount = 0;

  for (const Boid& otherBoid : boids) {
    double distance = currentPosition.distance(otherBoid.getPosition());
    if (distance > 0 && distance < m_perceptionRadius) {
      centerOfMass += otherBoid.getPosition();
      ++neighborCount;
    }
  }
  if (neighborCount > 0) {
    centerOfMass = centerOfMass / static_cast<double>(neighborCount);
    Vector cohesion = (centerOfMass - currentPosition) * cohesionFactor;
    cohesion.limit(m_maxSpeed);
    return cohesion;
  } else {
    return Vector{0., 0.};
  }
}

Vector Boid::align(std::vector<Boid> boids) const {
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
      ++neighborCount;
    }
  }
  if (neighborCount > 0) {
    averageVelocity = averageVelocity / static_cast<double>(neighborCount);
    Vector alignment = (averageVelocity - m_velocity) * alignmentFactor;
    alignment.limit(m_maxSpeed);
    return alignment;
  } else {
    return Vector{0.0, 0.0};
  }
}

void Boid::borders(unsigned int window_width, unsigned int window_height) {
  double halfWidth = window_width / 2;
  double halfHeight = window_height / 2;
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

/*
Vector Boid::centerOfMass(std::vector<Boid> boids)
    {
        if (boids.empty())
        {
            return Vector(0.0, 0.0);
        }
        int neighborCount = 0; // only close boids count in the formula not all
of them    Vector vSum(0.0, 0.0);    for (const Boid &other : boids)
        {
            double distance = m_position.distance(other.getPosition());
            if (distance > 0 && distance < m_perceptionRadius)
            {
                vSum = vSum + other.getPosition();
                ++neighborCount;
            }
        }
        if (neighborCount > 0)
        {
            vSum = vSum / neighborCount;
        }
        else
        {
            return m_position; // so in cohere desired is 0 with no neighbors
        }
        return vSum;
    }*/