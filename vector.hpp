#ifndef VECTOR_HPP
#define BOIDS_HPP
#include <vector>

class Vector {
  double m_x;
  double m_y;

 public:
  Vector(double x, double y) : m_x(x), m_y(y) {}

  Vector operator+(const Vector& v) const {}

  Vector operator-(const Vector& v) const {}

  Vector operator*(float scalar) const {}

  Vector operator/(float scalar) const {}

  double Magnitude() const {}

  Vector Normalize() const {}
};

#endif