#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <cmath>
#include <vector>

struct boidstate {  //position and components of the velocity of a single boid
  double x{}; 
  double y{};
  double vx{};
  double vy{};
};

class boid {
 private:
  double m_x{};
  double m_y{};
  double m_vx{};
  double m_vy{};

 public:
void separation();
void cohesion();
void alignment();

void update(); //for now i put void but i dont know if its right, update means it should give the boidstate given a certain time dt


};

#endif

/*
 public:
  BoidState getState() const {
    BoidState state;
    state.x = m_x;
    state.y = m_y;
    state.vx = m_vx;
    state.vy = m_vy;
    return state;
  }

  void setState(const BoidState& state) {
    m_x = state.x;
    m_y = state.y;
    m_vx = state.vx;
    m_vy = state.vy;
  }

  void separation(const std::vector<Boid>& boids) {
    // Calculate separation velocity
    double separationRadius = 20.0;  // Radius within which separation applies
    double separationFactor = 0.5;   // Separation influence factor

    double steerX = 0.0;
    double steerY = 0.0;
    int count = 0;

    for (const Boid& other : boids) {
      double dx = m_x - other.m_x;
      double dy = m_y - other.m_y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance > 0 && distance < separationRadius) {
        steerX += dx / distance;
        steerY += dy / distance;
        count++;
      }
    }

    if (count > 0) {
      steerX /= count;
      steerY /= count;
      double steerMagnitude = std::sqrt(steerX * steerX + steerY * steerY);
      steerX /= steerMagnitude;
      steerY /= steerMagnitude;
    }

    // Apply separation velocity
    m_vx += steerX * separationFactor;
    m_vy += steerY * separationFactor;
  }

  void cohesion(const std::vector<Boid>& boids) {
    // Calculate cohesion velocity
    double cohesionRadius = 50.0;   // Radius within which cohesion applies
    double cohesionFactor = 0.02;   // Cohesion influence factor

    double centerX = 0.0;
    double centerY = 0.0;
    int count = 0;

    for (const Boid& other : boids) {
      double dx = m_x - other.m_x;
      double dy = m_y - other.m_y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance > 0 && distance < cohesionRadius) {
        centerX += other.m_x;
        centerY += other.m_y;
        count++;
      }
    }

    if (count > 0) {
      centerX /= count;
      centerY /= count;
      double dx = centerX - m_x;
      double dy = centerY - m_y;
      double distance = std::sqrt(dx * dx + dy * dy);
      double cohesionX = dx / distance;
      double cohesionY = dy / distance;

      // Apply cohesion velocity
      m_vx += cohesionX * cohesionFactor;
      m_vy += cohesionY * cohesionFactor;
    }
  }

  void alignment(const std::vector<Boid>& boids) {
    // Calculate alignment velocity
    double alignmentRadius = 50.0;  // Radius within which alignment applies
    double alignmentFactor = 0.1;   // Alignment influence factor

    double avgVx = 0.0;
    double avgVy = 0.0;
    int count = 0;

    for (const Boid& other : boids) {
      double dx = m_x - other.m_x;
      double dy = m_y - other.m_y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance > 0 && distance < alignmentRadius) {
        avgVx += other.m_vx;
        avgVy += other.m_vy;
        count++;
      }
    }

    if (count > 0) {
      avgVx /= count;
      avgVy /= count;

      // Apply alignment velocity
      m_vx += (avgVx - m_vx) * alignmentFactor;
      m_vy += (avgVy - m_vy) * alignmentFactor;
    }
  }

  void update() {
    // Update boid's position based on its velocity
    m_x += m_vx;
    m_y += m_vy;
  }
};

int main() {
  std::vector<Boid> boids(10);  // Create a vector of 10 boids

  // Simulation loop
  for (int timestep = 0; timestep < 100; ++timestep) {
    // Update each boid
    for (Boid& boid : boids) {
      // Apply the three rules (separation, cohesion, alignment)
      boid.separation(boids);
      boid.cohesion(boids);
      boid.alignment(boids);

      // Update the boid's position
      boid.update();

      // Print the boid's current position
      BoidState state = boid.getState();
      std::cout << "Boid position: (" << state.x << ", " << state.y << ")" << std::endl;
    }

    // Print a separator between timesteps
    std::cout << "--------------" << std::endl;
  }

  return 0;
}*/