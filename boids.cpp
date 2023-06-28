#include "boids.hpp"
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>

/* ESEMPIO DI IMPLEMENTAZIONE

#include <cmath>
#include <iostream>
#include <vector>

struct Vector2D {
    float x;
    float y;
};

class Boid {
private:
    Vector2D position;
    Vector2D velocity;
    float maxSpeed;
    float perceptionRadius;

public:
    Boid(float x, float y, float maxSpeed, float perceptionRadius) {
        position.x = x;
        position.y = y;
        velocity.x = 0.0f;
        velocity.y = 0.0f;
        this->maxSpeed = maxSpeed;
        this->perceptionRadius = perceptionRadius;
    }

    void update(const std::vector<Boid>& boids) {
        Vector2D alignment = align(boids);
        Vector2D cohesion = cohere(boids);
        Vector2D separation = separate(boids);

        // Aggiorna la velocità in base alle regole dei Boids
        velocity.x += alignment.x + cohesion.x + separation.x;
        velocity.y += alignment.y + cohesion.y + separation.y;

        // Limita la velocità massima
        float speed = std::sqrt(velocity.x * velocity.x + velocity.y *
velocity.y); if (speed > maxSpeed) { velocity.x = (velocity.x / speed) *
maxSpeed; velocity.y = (velocity.y / speed) * maxSpeed;
        }

        // Aggiorna la posizione
        position.x += velocity.x;
        position.y += velocity.y;
    }

    Vector2D getPosition() const {
        return position;
    }

private:
    Vector2D align(const std::vector<Boid>& boids) {
        Vector2D steering;
        int total = 0;

        for (const auto& other : boids) {
            float distance = calculateDistance(position, other.position);

            if (&other != this && distance < perceptionRadius) {
                steering.x += other.velocity.x;
                steering.y += other.velocity.y;
                total++;
            }
        }

        if (total > 0) {
            steering.x /= total;
            steering.y /= total;

            float steeringMagnitude = std::sqrt(steering.x * steering.x +
steering.y * steering.y); steering.x = (steering.x / steeringMagnitude) *
maxSpeed - velocity.x; steering.y = (steering.y / steeringMagnitude) * maxSpeed
- velocity.y;

            float steeringMagnitude2 = std::sqrt(steering.x * steering.x +
steering.y * steering.y); if (steeringMagnitude2 > maxSpeed) { steering.x =
(steering.x / steeringMagnitude2) * maxSpeed; steering.y = (steering.y /
steeringMagnitude2) * maxSpeed;
            }
        }

        return steering;
    }

    Vector2D cohere(const std::vector<Boid>& boids) {
        Vector2D centerOfMass;
        int total = 0;

        for (const auto& other : boids) {
            float distance = calculateDistance(position, other.position);

            if (&other != this && distance < perceptionRadius) {
                centerOfMass.x += other.position.x;
                centerOfMass.y += other.position.y;
                total++;
            }
        }

        if (total > 0) {
            centerOfMass.x /= total;
            centerOfMass.y /= total;

            centerOfMass.x = (centerOfMass.x - position.x) / maxSpeed;
            centerOfMass.y = (centerOfMass.y - position.y) / maxSpeed;

            float centerMagnitude = std::sqrt(centerOfMass.x * centerOfMass.x +
centerOfMass.y * centerOfMass.y); centerOfMass.x = (centerOfMass.x /
centerMagnitude) * maxSpeed - velocity.x; centerOfMass.y = (centerOfMass.y /
centerMagnitude) * maxSpeed - velocity.y;

            float centerMagnitude2 = std::sqrt(centerOfMass.x * centerOfMass.x +
centerOfMass.y * centerOfMass.y); if (centerMagnitude2 > maxSpeed) {
                centerOfMass.x = (centerOfMass.x / centerMagnitude2) * maxSpeed;
                centerOfMass.y = (centerOfMass.y / centerMagnitude2) * maxSpeed;
            }
        }

        return centerOfMass;
    }

    Vector2D separate(const std::vector<Boid>& boids) {
        Vector2D steering;
        int total = 0;

        for (const auto& other : boids) {
            float distance = calculateDistance(position, other.position);

            if (&other != this && distance < perceptionRadius) {
                float diffX = position.x - other.position.x;
                float diffY = position.y - other.position.y;

                if (distance > 0) {
                    steering.x += diffX / distance;
                    steering.y += diffY / distance;
                }

                total++;
            }
        }

        if (total > 0) {
            steering.x /= total;
            steering.y /= total;

            float steeringMagnitude = std::sqrt(steering.x * steering.x +
steering.y * steering.y); steering.x = (steering.x / steeringMagnitude) *
maxSpeed - velocity.x; steering.y = (steering.y / steeringMagnitude) * maxSpeed
- velocity.y;

            float steeringMagnitude2 = std::sqrt(steering.x * steering.x +
steering.y * steering.y); if (steeringMagnitude2 > maxSpeed) { steering.x =
(steering.x / steeringMagnitude2) * maxSpeed; steering.y = (steering.y /
steeringMagnitude2) * maxSpeed;
            }
        }

        return steering;
    }

    float calculateDistance(const Vector2D& v1, const Vector2D& v2) const {
        float diffX = v1.x - v2.x;
        float diffY = v1.y - v2.y;
        return std::sqrt(diffX * diffX + diffY * diffY);
    }
};

int main() {
    const int numBoids = 10;
    const int screenWidth = 800;
    const int screenHeight = 600;

    std::vector<Boid> boids;

    for (int i = 0; i < numBoids; ++i) {
        float x = static_cast<float>(rand() % screenWidth);
        float y = static_cast<float>(rand() % screenHeight);
        float maxSpeed = 3.0f;
        float perceptionRadius = 50.0f;
        boids.emplace_back(x, y, maxSpeed, perceptionRadius);
    }

    // Simulazione
    for (int frame = 0; frame < 100; ++frame) {
        // Aggiorna ogni boid
for (auto& boid : boids) {
        // Crea una copia dei boids escludendo il boid corrente
        std::vector<Boid> otherBoids = boids;
        otherBoids.erase(std::remove(otherBoids.begin(), otherBoids.end(),
boid), otherBoids.end());

        // Aggiorna la boid corrente
        boid.update(otherBoids);

        // Ottieni la posizione aggiornata della boid corrente
        Vector2D position = boid.getPosition();
        std::cout << "Boid position: " << position.x << ", " << position.y <<
std::endl;
    }
 }
*/




/* ALTRO ESEMPIO:
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

// Definizione della classe Boid
class Boid {
public:
    double x, y;  // Posizione
    double vx, vy;  // Velocità

    Boid(double posX, double posY, double velX, double velY)
        : x(posX), y(posY), vx(velX), vy(velY) {}
};

// Funzione per calcolare la distanza tra due boids
double distance(const Boid& boid1, const Boid& boid2) {
    return std::hypot(boid2.x - boid1.x, boid2.y - boid1.y);
}

// Funzione per calcolare la coesione di un boid
void cohesion(const Boid& currentBoid, const std::vector<Boid>& boids, double&
cx, double& cy) { int count = 0; cx = 0.0; cy = 0.0;

    for (const Boid& boid : boids) {
        double d = distance(currentBoid, boid);
        if (&currentBoid != &boid && d < 100.0) {
            cx += boid.x;
            cy += boid.y;
            count++;
        }
    }

    if (count > 0) {
        cx /= count;
        cy /= count;
        cx = (cx - currentBoid.x) / 100.0;
        cy = (cy - currentBoid.y) / 100.0;
    }
}

// Funzione per calcolare l'allineamento di un boid
void alignment(const Boid& currentBoid, const std::vector<Boid>& boids, double&
ax, double& ay) { int count = 0; ax = 0.0; ay = 0.0;

    for (const Boid& boid : boids) {
        double d = distance(currentBoid, boid);
        if (&currentBoid != &boid && d < 100.0) {
            ax += boid.vx;
            ay += boid.vy;
            count++;
        }
    }

    if (count > 0) {
        ax /= count;
        ay /= count;
        ax = (ax - currentBoid.vx) / 8.0;
        ay = (ay - currentBoid.vy) / 8.0;
    }
}

// Funzione per calcolare la separazione di un boid
void separation(const Boid& currentBoid, const std::vector<Boid>& boids, double&
sx, double& sy) { sx = 0.0; sy = 0.0;

    for (const Boid& boid : boids) {
        double d = distance(currentBoid, boid);
        if (&currentBoid != &boid && d < 30.0) {
            sx -= (boid.x - currentBoid.x);
            sy -= (boid.y - currentBoid.y);
        }
    }
}

// Funzione per aggiornare la posizione e la velocità di un boid
void updateBoid(Boid& currentBoid, const std::vector<Boid>& boids) {
    double cx, cy;  // Coesione
    double ax, ay;  // Allineamento
    double sx, sy;  // Separazione

    cohesion(currentBoid, boids, cx, cy);
    alignment(currentBoid, boids, ax, ay);
    separation(currentBoid, boids, sx, sy);

    // Aggiornamento della velocità
    currentBoid.vx += cx + ax + sx;
    currentBoid.vy += cy + ay + sy;

    // Limitazione della velocità massima
    double speedLimit = 5.0;
    double speed = std::hypot(currentBoid.vx, currentBoid.vy);
    if (speed > speedLimit) {
        currentBoid.vx = (currentBoid.vx / speed) * speedLimit;
        currentBoid.vy = (currentBoid.vy / speed) * speedLimit;
    }

    // Aggiornamento della posizione
    currentBoid.x += currentBoid.vx;
    currentBoid.y += currentBoid.vy;
}

int main() {
    const int numBoids = 100;
    std::vector<Boid> boids;

    // Inizializzazione dei boids con posizioni e velocità casuali
    for (int i = 0; i < numBoids; i++) {
        double posX = std::rand() % 800 + 100;
        double posY = std::rand() % 600 + 100;
        double velX = (std::rand() % 20 - 10) / 10.0;
        double velY = (std::rand() % 20 - 10) / 10.0;
        boids.push_back(Boid(posX, posY, velX, velY));
    }

    // Simulazione del comportamento dello stormo
    for (int t = 0; t < 1000; t++) {
        // Aggiornamento dei boids
        for (Boid& boid : boids) {
            updateBoid(boid, boids);
        }

        // Stampa delle posizioni dei boids
        for (const Boid& boid : boids) {
            std::cout << "Boid at (" << boid.x << ", " << boid.y << ")" <<
std::endl;
        }

        std::cout << "----------------------" << std::endl;
    }

    return 0;
}

*/