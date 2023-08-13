#include "flocktelly.hpp"

Flock::Flock()
    : m_boids{
          std::vector<Boid>()} {};  // constructor, empty container of Boids.
std::vector<Boid> Flock::getmboids() { return m_boids; }
int Flock::getnumboids() { return m_boids.size(); }
void Flock::addBoid(const Boid &boid) { m_boids.push_back(boid); }
void Flock::removeBoid(const Boid &boid) {
  auto it = std::find(m_boids.begin(), m_boids.end(), boid);
    m_boids.erase(it);
}
void Flock::updateVelocity() {
  for (Boid &b : m_boids) {
    Vector separatioVelocity = b.separate(m_boids);
    Vector cohesionVelocity = b.cohere(m_boids);
    Vector alignmentVelocity = b.align(m_boids);
    Vector totalVelocity =
        separatioVelocity + cohesionVelocity + alignmentVelocity;
    b.setVelocity(b.vel().xcomp() + totalVelocity.xcomp(),
                  b.vel().ycomp() + totalVelocity.ycomp());
    if (b.speed() > b.maxSpeed()) {
      b.vel() =
          (b.maxSpeed() / b.speed()) *
          b.vel();  // it may be better than creating the function in Vector
    }
  }
}
void Flock::updatePosition() {
  updateVelocity();//????????????????
  for (Boid &b : m_boids) {
    Vector vSum = delta_time * b.vel();
    b.setPosition(b.pos().xcomp() + vSum.xcomp(),
                  b.pos().ycomp() + vSum.ycomp());
  }
}
double Flock::averageDistance() const {
  if (m_boids.empty()) {
    return 0.0;
  }
  double vDist = 0.0;
  for (const Boid &b : m_boids) {
    vDist += distance(b.pos(), b.centerOfMass(m_boids));
  }
  return vDist / m_boids.size();
}

double Flock::averageSpeed() {
  if (m_boids.empty()) {
    return 0.0;
  }
  double vSpeed = 0.0;
  for (const Boid &b : m_boids) {
    vSpeed += b.speed();
  }
  return vSpeed / m_boids.size();
}
double Flock::standardDeviationDistance() {
  if (m_boids.size() < 2) {
    return 0.0;
  }
  double avgDistance = averageDistance();
  double stDev = 0.0;
  for (const Boid &b : m_boids) {
    double diff = distance(b.pos(), b.centerOfMass(m_boids)) - avgDistance;
    stDev += diff * diff;
  }
  return std::sqrt(stDev / m_boids.size() - 1);
}
double Flock::standardDeviationSpeed() {
  if (m_boids.size() < 2) {
    return 0.0;
  }
  double avgSpeed = averageSpeed();
  double stDev = 0.0;
  for (const Boid &b : m_boids) {
    double diff = b.speed() - avgSpeed;
    stDev += diff * diff;
  }
  return std::sqrt(stDev / m_boids.size() - 1);
}
void Flock::simulate(int numSteps) {
  for (int step = 0; step < numSteps;
       step++)  // at each step updates and prints the collective infotmations
                // about the flock.
  {
    updateVelocity();
    updatePosition();
    double time = step * delta_time;
    std::cout << "Time: " << time << std::endl;
    std::cout << "Average distance: " << averageDistance() << std::endl;
    std::cout << "Average speed: " << averageSpeed() << std::endl;
    std::cout << "Standard deviation of distance: "
              << standardDeviationDistance() << std::endl;
    std::cout << "Standard deviation of speed: " << standardDeviationSpeed()
              << std::endl;
  }
}