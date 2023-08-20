#include "flock.hpp"
namespace fk
{
    Flock::Flock() : m_boids{std::vector<bd::Boid>()} {}; // constructor, empty container of Boids.
    void Flock::addBoid(const bd::Boid &boid)
    {
        m_boids.push_back(boid);
    }
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
        std::vector<Vector> newVelocities(m_boids.size()); // create a new vector to pud the updated velocities

        for (size_t i = 0; i < m_boids.size(); ++i)
        {
            bd::Boid &b = m_boids[i];

            Vector separatioVelocity = b.separate(m_boids);
            Vector cohesionVelocity = b.cohere(m_boids);
            Vector alignmentVelocity = b.align(m_boids);
            Vector totalVelocity = separatioVelocity + cohesionVelocity + alignmentVelocity;
            Vector newVelocity = b.vel() + totalVelocity; // not setting it yet because then the boids are not updated at the same time and updated boids influence non updated boids
            if (magnitude(newVelocity) > b.maxSpeed())
            {
                newVelocity = (b.maxSpeed() / magnitude(newVelocity)) * newVelocity;
            }
            newVelocities[i] = newVelocity; // put the updated velocities in the new vector
        }
        for (size_t i = 0; i < m_boids.size(); ++i)
        {
            m_boids[i].setVelocity(newVelocities[i].xcomp(), newVelocities[i].ycomp()); // now we can safley set
        }
    }
    void Flock::updatePosition(unsigned int windowWidth, unsigned int windowHeight) // same reasoning as for updateVelocity
    {
        updateVelocity();
        std::vector<Vector> newPositions(m_boids.size());
        for (size_t i = 0; i < m_boids.size(); ++i)
        {
            bd::Boid &b = m_boids[i];
            Vector newPosition = b.pos() + delta_time * b.vel();
            newPositions[i] = newPosition;
        }
        for (size_t i = 0; i < m_boids.size(); ++i)
        {
            m_boids[i].setPosition(newPositions[i].xcomp(), newPositions[i].ycomp());
            m_boids[i].border(windowWidth, windowHeight);
        }
    }
    double Flock::averageDistance()
    {
        if (m_boids.empty())
        {
            return 0.0;
        }
        double vDist = 0.0;
        double nPairs = 0.0;
        for (size_t i = 0; i < m_boids.size(); ++i)
        {
            for (size_t j = i + 1; j < m_boids.size(); ++j)
            {
                double dist = distance(m_boids[i].pos(), m_boids[j].pos());
                vDist += dist;
                ++nPairs;
            }
        }
        if (nPairs == 0)
        {
            return 0.0;
        }
        return vDist / nPairs;
    }

    double Flock::averageSpeed()
    {
        if (m_boids.empty())
        {
            return 0.0;
        }
        double vSpeed = 0.0;
        for (const bd::Boid &b : m_boids)
        {
            vSpeed += b.speed();
        }
        return vSpeed / m_boids.size();
    }
    double Flock::standardDeviationDistance()
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
                double dist = distance(m_boids[i].pos(), m_boids[j].pos());
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
    double Flock::standardDeviationSpeed()
    {
        if (m_boids.size() < 2)
        {
            return 0.0;
        }
        double avgSpeed = averageSpeed();
        double stDev = 0.0;
        for (const bd::Boid &b : m_boids)
        {
            double diff = b.speed() - avgSpeed;
            stDev += diff * diff;
        }
        return std::sqrt(stDev / m_boids.size());
    }
    void Flock::simulate(int numSteps, unsigned int windowWidth, unsigned int windowHeight)
    {
        for (int step = 0; step <= numSteps; step++) // at each step updates and prints the collective infotmations about the flock.
        {
            double time = step * delta_time;
            std::cout << "Time: " << time << " seconds" << std::endl;
            std::cout << "Average distance: " << averageDistance() << std::endl;
            std::cout << "Average speed: " << averageSpeed() << std::endl;
            std::cout << "Standard deviation of distance: " << standardDeviationDistance() << std::endl;
            std::cout << "Standard deviation of speed: " << standardDeviationSpeed() << std::endl;
            updateVelocity();
            updatePosition(windowWidth, windowHeight);
        }
    }
}