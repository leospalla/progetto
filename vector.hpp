#ifndef VECTOR_HPP
#define VECTOR_HPP
#include <vector>

class Vector {
  double x;
  double y;

 public:
  Vector(double xp, double yp) : x(xp), y(yp) {}

  void add(const Vector& v);

  void multiply(const Vector& v);

  void subtract(const Vector& v);

  void mulScalar(double scalar);

  void divScalar(double scalar);

  double Magnitude();

  void Normalize();

  double dotProduct(const Vector& v);
};

#endif