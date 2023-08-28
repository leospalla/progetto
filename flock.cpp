#include "flock.hpp"
namespace fk
{
  // constructor, empty container of Boids
  Flock::Flock() : m_boids{std::vector<bd::Boid>()} {}

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
    // creates a new vector to put the updated velocities
    std::vector<vc::Vector> newVelocities(m_boids.size());
    for (size_t i = 0; i < m_boids.size(); ++i)
    {
      bd::Boid &b = m_boids[i];
      vc::Vector separationVelocity = b.separate(m_boids);
      vc::Vector cohesionVelocity = b.cohere(m_boids);
      vc::Vector alignmentVelocity = b.align(m_boids);
      vc::Vector totalVelocity =
          separationVelocity + cohesionVelocity + alignmentVelocity;
      // not setting it yet because then the boids are not
      // updated at the same time and updated boids influence non-updated boids
      vc::Vector newVelocity = b.getVelocity() + totalVelocity;
      newVelocity.limit(b.getMaxSpeed());
      newVelocities[i] = newVelocity;
    }
    for (size_t j = 0; j < m_boids.size(); ++j)
    {
      // now we can safely set the new velocities
      m_boids[j].setVelocity(newVelocities[j].getX(), newVelocities[j].getY());
    }
  }

  // same thing for updateVelocity
  void Flock::updatePosition(int size)
  {
    updateVelocity();
    std::vector<vc::Vector> newPositions(m_boids.size());
    for (size_t i = 0; i < m_boids.size(); ++i)
    {
      bd::Boid &b = m_boids[i];
      vc::Vector newPosition = b.getPosition() + b.getVelocity() * m_delta_time;
      newPositions[i] = newPosition;
    }
    for (size_t j = 0; j < m_boids.size(); ++j)
    {
      m_boids[j].setPosition(newPositions[j].getX(), newPositions[j].getY());
      m_boids[j].borders(size);
    }
  }

  void Flock::updateBoidParameters(double perceptionRadius,
                                   double separationDistance,
                                   double separationFactor, double cohesionFactor,
                                   double alignmentFactor)
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

  double Flock::averageDistance() const
  {
    if (m_boids.empty())
    {
      return 0.;
    }
    double nPairs = static_cast<double>(m_boids.size() * (m_boids.size() - 1)) / 2.0;
    auto calculateTotalDistance = [&](double sumDistance, const bd::Boid &boidI)
    {
      const vc::Vector &posI = boidI.getPosition();
      return std::accumulate(m_boids.begin(), m_boids.end(), sumDistance,
                             [&](double innerSum, const bd::Boid &boidJ)
                             {
                               double dist = posI.distance(boidJ.getPosition());
                               return innerSum + dist;
                             });
    };
    double totalDistance = std::accumulate(m_boids.begin(), m_boids.end(), 0.0, calculateTotalDistance); // counts same distance twice
    totalDistance /= 2;
    if (nPairs == 0)
    {
      return 0.;
    }
    return totalDistance / nPairs;
  }

  double Flock::averageSpeed() const
  {
    if (m_boids.empty())
    {
      return 0.0;
    }
    auto vSpeed = [&](const double vSum, const bd::Boid &b)
    {
      return vSum + b.getSpeed();
    };
    double totalSpeed = std::accumulate(m_boids.begin(), m_boids.end(), 0.0, vSpeed);
    return totalSpeed / m_boids.size();
  }

  double Flock::standardDeviationDistance() const
  {
    if (m_boids.size() < 2)
    {
      return 0.0;
    }

    double avgDistance = averageDistance();
    double nPairs = static_cast<double>(m_boids.size() * (m_boids.size() - 1)) / 2.0;

    auto calculateDifferenceSum = [&](double sumDiffDistance, const bd::Boid &boidI)
    {
      const vc::Vector &posI = boidI.getPosition();
      return std::accumulate(m_boids.begin(), m_boids.end(), sumDiffDistance,
                             [&](double sum, const bd::Boid &boidJ)
                             {
                               double dist = posI.distance(boidJ.getPosition());
                               double diff = dist - avgDistance;
                               return sum + diff * diff;
                             });
    };

    double differenceSum = std::accumulate(m_boids.begin(), m_boids.end(), 0.0, calculateDifferenceSum);

    if (nPairs == 0)
    {
      return 0.0;
    }

    return std::sqrt(differenceSum / nPairs) / 4;
  }

  double Flock::standardDeviationSpeed() const
  {
    if (m_boids.size() < 2)
    {
      return 0.;
    }
    double avgSpeed = averageSpeed();
    auto calculateStDev = [&](const double stDev, const bd::Boid &b)
    {
      double diff = b.getSpeed() - avgSpeed;
      return stDev + diff * diff;
    };
    double standardDeviation = std::accumulate(m_boids.begin(), m_boids.end(), 0.0, calculateStDev);
    return std::sqrt(standardDeviation / m_boids.size());
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
              if (!visited[j] && m_boids[j].getPosition().distance(
                                     m_boids[currentBoidIndex].getPosition()) <=
                                     perceptionRadius)
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

  // at each step updates and prints the collective informations about the flock.
  void Flock::simulate(int numSteps, int size, double perceptionRadius)
  {
    for (int step = 0; step <= numSteps; ++step)
    {
      double time = step * m_delta_time;
      std::cout << "Time: " << time << std::endl;
      std::cout << "Average distance: " << averageDistance() << " +/- "
                << standardDeviationDistance() << std::endl;
      std::cout << "Average speed: " << averageSpeed() << " +/- "
                << standardDeviationSpeed() << std::endl;
      std::vector<int> flockCounts = countBoidsInFlock(perceptionRadius);
      std::cout << "Number of flocks: " << flockCounts.size() << std::endl;
      for (size_t i = 0; i < flockCounts.size(); ++i)
      {
        std::cout << "Number of boids inside flock n°" << i + 1 << ": "
                  << flockCounts[i] << std::endl;
      }
      updateVelocity();
      updatePosition(size);
    }
  }
} // namespace fk