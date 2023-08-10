#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <vector>
#include <random>
#include <stdexcept>
#include <iostream>

#include "vector.hpp"

class Boid {
 private:
  Vector m_position{};
  Vector m_velocity{};
  Vector m_acceleration{0., 0.};
  double const m_maxSpeed{5.};  // here we should put a default
  double m_perceptionRadius;    // this needs to be taken as input

  // should we add a class invariant? (it needs to go in the public anyway)

public:
double const separationDistance{1.};  // must be less than the perception radius
double const separationFactor{1.};
double const cohesionFactor{1.};
double const alignmentFactor{0.5};  // must be less than 1

Boid();                // default constructor
Boid(double, double);  // constructor with position coordinates
Boid(Vector);          // constructor with position vector
Boid(Vector, Vector);  //constructor with both vectors

Vector getPosition() const { return m_position; }
Vector getVelocity() const { return m_velocity; }
Vector getAcceleration() const { return m_acceleration; }

void setPosition(double, double);
void setVelocity(double, double);

double getSpeed();

  Vector separate(std::vector<Boid>);

  Vector cohere(std::vector<Boid>);

  Vector align(std::vector<Boid>);

void update(std::vector<Boid>); 
void borders();
};

#endif

/*
{
    // testing using perceptionRadius = 10. , alignFactor = 0.5

    std::vector<Boid> boids;

    Boid b1{};
    Boid b2{100., 0.};

    b1.setPosition(0., 0.);
    b1.setVelocity(3., 2.);
    b2.setPosition(2., 0.);
    b2.setVelocity(0., 4.);
    boids.push_back(b1);
    boids.push_back(b2);
    b1.update(boids);
    // these dont work idk why
    // through debugging i found out that for some reason in the flow control of
    // the align function, otherBoid.getVelocity() is a (4,2) vector

    CHECK(b1.getVelocity().x == doctest::Approx(1.5));
    CHECK(b1.getVelocity().y == doctest::Approx(3.5));
    CHECK(b1.getPosition().x == doctest::Approx(1.5));
    CHECK(b1.getPosition().y == doctest::Approx(3.5));
  }
  
  */