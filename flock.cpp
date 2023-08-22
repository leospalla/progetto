#include "flock.hpp"
namespace fk {
// constructor, empty container of Boids
Flock::Flock() : m_boids{std::vector<bd::Boid>()} {}

void Flock::addBoid(const bd::Boid &boid) { m_boids.push_back(boid); }

void Flock::removeBoid(const bd::Boid &boid) {
  auto it = std::find(m_boids.begin(), m_boids.end(), boid);
  if (it != m_boids.end()) {
    m_boids.erase(it);
  }
}

void Flock::updateVelocity() {
  // create a new vector to put the updated velocities
  std::vector<vc::Vector> newVelocities(m_boids.size());
  for (size_t i = 0; i < m_boids.size(); ++i) {
    bd::Boid &b = m_boids[i];
    vc::Vector separationVelocity = b.separate(m_boids);
    vc::Vector cohesionVelocity = b.cohere(m_boids);
    vc::Vector alignmentVelocity = b.align(m_boids);
    vc::Vector totalVelocity =
        separationVelocity + cohesionVelocity + alignmentVelocity;
    // not setting it yet because then the boids are not
    // updated at the same time and updated boids influence non updated boids
    vc::Vector newVelocity = b.getVelocity() + totalVelocity;
    newVelocity.limit(b.getMaxSpeed());
    newVelocities[i] = newVelocity;
  }
  for (size_t j = 0; j < m_boids.size(); ++j) {
    // now we can safely set the new velocities
    m_boids[j].setVelocity(newVelocities[j].getX(), newVelocities[j].getY());
  }
}

void Flock::updatePosition(unsigned int windowWidth,
                           unsigned int windowHeight) {
  updateVelocity();
  std::vector<vc::Vector> newPositions(m_boids.size());
  for (size_t i = 0; i < m_boids.size(); ++i) {
    bd::Boid &b = m_boids[i];
    vc::Vector newPosition = b.getPosition() + b.getVelocity() * m_delta_time;
    newPositions[i] = newPosition;
  }
  for (size_t j = 0; j < m_boids.size(); ++j) {
    m_boids[j].setPosition(newPositions[j].getX(), newPositions[j].getY());
    m_boids[j].borders(windowWidth, windowHeight);
  }
}

void Flock::updateBoidParameters(double perceptionRadius,
                                 double separationDistance,
                                 double separationFactor, double cohesionFactor,
                                 double alignmentFactor) {
  for (bd::Boid &boid : m_boids) {
    boid.setPerceptionRadius(perceptionRadius);
    boid.setSeparationDistance(separationDistance);
    boid.setSeparationFactor(separationFactor);
    boid.setCohesionFactor(cohesionFactor);
    boid.setAlignmentFactor(alignmentFactor);
  }
}

double Flock::averageDistance() const {
  if (m_boids.empty()) {
    return 0.;
  }
  double totalDistance = 0.;
  double nPairs = 0.;
  for (size_t i = 0; i < m_boids.size() - 1; ++i) {
    const vc::Vector &posI = m_boids[i].getPosition();
    for (size_t j = i + 1; j < m_boids.size(); ++j) {
      double dist = posI.distance(m_boids[j].getPosition());
      totalDistance += dist;
      ++nPairs;
    }
  }
  if (nPairs == 0) {
    return 0.;
  }
  return totalDistance / nPairs;
}

double Flock::averageSpeed() const {
  if (m_boids.empty()) {
    return 0.0;
  }
  double vSpeed = 0.;
  for (const bd::Boid &b : m_boids) {
    vSpeed += b.getSpeed();
  }
  return vSpeed / m_boids.size();
}

double Flock::standardDeviationDistance() const {
  if (m_boids.size() < 2) {
    return 0.0;
  }
  double avgDistance = averageDistance();
  double stDev = 0.0;
  double nPairs = 0.0;
  for (size_t i = 0; i < m_boids.size(); ++i) {
    for (size_t j = i + 1; j < m_boids.size(); ++j) {
      double dist = m_boids[i].getPosition().distance(m_boids[j].getPosition());
      double diff = dist - avgDistance;
      stDev += diff * diff;
      ++nPairs;
    }
  }
  if (nPairs == 0) {
    return 0.;
  }
  return std::sqrt(stDev / nPairs);
}

double Flock::standardDeviationSpeed() const {
  if (m_boids.size() < 2) {
    return 0.;
  }
  double avgSpeed = averageSpeed();
  double stDev = 0.;
  for (const bd::Boid &b : m_boids) {
    double diff = b.getSpeed() - avgSpeed;
    stDev += diff * diff;
  }
  return std::sqrt(stDev / m_boids.size());
}

std::vector<int> Flock::countBoidsInFlock() const {
  std::vector<int> empty;
  std::vector<int> one(1, 1);
  if (m_boids.size() == 0) {
    return empty;
  }
  if (m_boids.size() == 1) {
    return one;
  }
  std::vector<bool> visited(m_boids.size(), false);
  std::vector<int> memberCounts;
  bd::Boid b;
  int numBoids{0};
  int numFlocks{0};
  int oldNumFlocks{0};
  for (size_t i{0}; i < m_boids.size(); ++i) {
    int sameflk{0};
    if (!visited[i]) {
      visited[i] = true;
      oldNumFlocks = numFlocks;
      ++numBoids;
      ++numFlocks;
      for (size_t k{0}; k < m_boids.size(); ++k) {
        if (k != i && visited[k] &&
            m_boids[i].getPosition().distance(m_boids[k].getPosition()) <=
                b.perceptionRadius) {
          ++sameflk;
        }
      }
      if (sameflk != 0) {
        --numFlocks;
      }
      if (numBoids != 1 && oldNumFlocks != numFlocks) {
        memberCounts.push_back(numBoids - 1);
        numBoids = 1;
      }
      for (size_t j{i + 1}; j < m_boids.size(); ++j) {
        if (!visited[j] &&
            m_boids[i].getPosition().distance(m_boids[j].getPosition()) <=
                b.perceptionRadius) {
          visited[j] = true;
          ++numBoids;
        }
      }
      if (numBoids == 1) {
        memberCounts.push_back(numBoids);
        numBoids = 0;
      }
    }
  }
  if (numBoids != 0) {
    memberCounts.push_back(numBoids);
  }
  return memberCounts;
}

// at each step updates and prints the collective informations about the flock.
void Flock::simulate(int numSteps, [[maybe_unused]] unsigned int windowWidth,
                     [[maybe_unused]] unsigned int windowHeight) {
  for (int step = 0; step <= numSteps; ++step) {
    double time = step * m_delta_time;
    std::cout << "Time: " << time << std::endl;
    std::cout << "Average distance: " << averageDistance() << " +/- "
              << standardDeviationDistance() << std::endl;
    std::cout << "Average speed: " << averageSpeed() << " +/- "
              << standardDeviationSpeed() << std::endl;
    std::cout << "Number of flocks: " << countBoidsInFlock().size()
              << std::endl;
    for (size_t i{0}; i < countBoidsInFlock().size(); ++i) {
      std::cout << "Number of boids inside flock n°" << i + 1 << " : "
                << countBoidsInFlock()[i] << std::endl;
    }
    updateVelocity();
    updatePosition(windowHeight, windowHeight);
  }
}
}  // namespace fk