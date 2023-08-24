#include "flock.hpp"
namespace fk
{
  // constructor, empty container of Boids
  Flock::Flock() : m_boids{std::vector<bd::Boid>()} {};

  void Flock::addBoid(const bd::Boid &boid) { m_boids.push_back(boid); }

  void Flock::removeBoid(const bd::Boid &boid)
  {
    auto it = std::find(m_boids.begin(), m_boids.end(), boid);
    if (it != m_boids.end())
    {
      m_boids.erase(it);
    }
  }

  void Flock::updateVelocity()
  {
    // create a new vector to put the updated velocities
    std::vector<vc::Vector> newVelocities(m_boids.size());
    for (size_t i = 0; i < m_boids.size(); ++i)
    {
      bd::Boid &b = m_boids[i];
      vc::Vector separationVelocity = b.separate(m_boids);
      vc::Vector cohesionVelocity = b.cohere(m_boids);
      vc::Vector alignmentVelocity = b.align(m_boids);
      vc::Vector totalVelocity = separationVelocity + cohesionVelocity + alignmentVelocity;
      vc::Vector newVelocity = b.getVelocity() + totalVelocity; // not setting it yet because then the boids are not updated at the same time and updated boids influence non updated boids
      if (newVelocity.Magnitude() > b.getMaxSpeed())
      {
        newVelocity = newVelocity * (b.getMaxSpeed() / newVelocity.Magnitude());
      }
      newVelocities[i] = newVelocity; // put the updated velocities in the new vector
    }
    for (size_t i = 0; i < m_boids.size(); ++i)
    {
      m_boids[i].setVelocity(newVelocities[i].getX(), newVelocities[i].getY()); // now we can safley set
    }
  }

  void Flock::updatePosition(unsigned int windowWidth, unsigned int windowHeight)
  {
    updateVelocity();
    std::vector<vc::Vector> newPositions(m_boids.size());
    for (size_t i = 0; i < m_boids.size(); ++i)
    {
      bd::Boid &b = m_boids[i];
      vc::Vector newPosition = b.getPosition() + b.getVelocity() * m_deltaTime;
      newPositions[i] = newPosition;
    }
    for (size_t i = 0; i < m_boids.size(); ++i)
    {
      m_boids[i].setPosition(newPositions[i].getX(), newPositions[i].getY());
      m_boids[i].borders(windowWidth, windowHeight);
    }
  }

  void Flock::updateBoidParameters(double perceptionRadius, double separationDistance, double separationFactor, double cohesionFactor, double alignmentFactor)
  {
    for (bd::Boid &boid : m_boids)
    {
      boid.setPerceptionRadius(perceptionRadius);
      boid.setSeparationDistance(separationDistance);
      boid.setSeparationFactor(separationFactor);
      boid.setCohesionFactor(cohesionFactor);
      boid.setAlignmentFactor(alignmentFactor);
    }
  }

  std::vector<int> Flock::countBoidsInFlock(double perceptionRadius) const
  {
    std::vector<int> memberCounts;
    std::vector<bool> visited(m_boids.size(), false);

    for (size_t i = 0; i < m_boids.size(); ++i)
    {
      if (!visited[i])
      {
        int numBoidsInFlock = 0;
        std::stack<size_t> stack;
        stack.push(i);

        while (!stack.empty())
        {
          size_t currentBoidIndex = stack.top();
          stack.pop();
          if (!visited[currentBoidIndex])
          {
            visited[currentBoidIndex] = true;
            ++numBoidsInFlock;

            for (size_t j = 0; j < m_boids.size(); ++j)
            {
              if (!visited[j] && m_boids[j].getPosition().distance(m_boids[currentBoidIndex].getPosition()) <= perceptionRadius)
              {
                stack.push(j);
              }
            }
          }
        }

        if (numBoidsInFlock > 0)
        {
          memberCounts.push_back(numBoidsInFlock);
        }
      }
    }

    return memberCounts;
  }

  double Flock::averageDistance() const
  {
    if (m_boids.empty())
    {
      return 0.0;
    }
    double totalDistance = 0.0;
    double nPairs = 0.0;
    for (size_t i = 0; i < m_boids.size() - 1; ++i)
    {
      const vc::Vector &posI = m_boids[i].getPosition();
      for (size_t j = i + 1; j < m_boids.size(); ++j)
      {
        double dist = posI.distance(m_boids[j].getPosition());
        totalDistance += dist;
        ++nPairs;
      }
    }
    if (nPairs == 0)
    {
      return 0.0;
    }
    return totalDistance / nPairs;
  }

  double Flock::averageSpeed() const
  {
    if (m_boids.empty())
    {
      return 0.0;
    }
    double vSpeed = 0.0;
    for (const bd::Boid &b : m_boids)
    {
      vSpeed += b.getSpeed();
    }
    return vSpeed / m_boids.size();
  }

  double Flock::standardDeviationDistance() const
  {
    if (m_boids.size() < 2)
    {
      return 0.0;
    }
    double avgDistance = averageDistance();
    double stDev = 0.0;
    double nPairs = 0.0;
    for (size_t i = 0; i < m_boids.size(); ++i)
    {
      for (size_t j = i + 1; j < m_boids.size(); ++j)
      {
        double dist = m_boids[i].getPosition().distance(m_boids[j].getPosition());
        double diff = dist - avgDistance;
        stDev += diff * diff;
        ++nPairs;
      }
    }
    if (nPairs == 0)
    {
      return 0.0;
    }
    return std::sqrt(stDev / nPairs);
  }

  double Flock::standardDeviationSpeed() const
  {
    if (m_boids.size() < 2)
    {
      return 0.0;
    }
    double avgSpeed = averageSpeed();
    double stDev = 0.0;
    for (const bd::Boid &b : m_boids)
    {
      double diff = b.getSpeed() - avgSpeed;
      stDev += diff * diff;
    }
    return std::sqrt(stDev / m_boids.size());
  }

  // at each step updates and prints the collective informations about the flock.
  void Flock::simulate(int numSteps, unsigned int windowWidth, unsigned int windowHeight, double perceptionRadius)
  {
    for (int step = 0; step <= numSteps; ++step)
    {
      std::vector<int> memberCounts;
      memberCounts = countBoidsInFlock(perceptionRadius);
      double time = step * m_deltaTime;
      std::cout << "Time: " << time << " seconds" << std::endl;
      std::cout << "Average distance: " << averageDistance() << " +/- " << standardDeviationDistance() << std::endl;
      std::cout << "Average speed: " << averageSpeed() << " +/- " << standardDeviationSpeed() << std::endl;
      std::cout << "number of flocks " << memberCounts.size() << std::endl;
      for (size_t i{0}; i < memberCounts.size(); ++i)
      {
        std::cout << "the number of boids in flock: " << i + 1 << " is: " << memberCounts[i] << std::endl;
      }
      updateVelocity();
      updatePosition(windowHeight, windowHeight);
    }
  }
} // namespace fk