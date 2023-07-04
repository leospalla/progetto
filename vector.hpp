#ifndef VECTOR_HPP
#define VECTOR_HPP
#include <cmath>

class Vector {
 public:
  double x;
  double y;

  Vector(double xp, double yp) : x{xp}, y{yp} {}  // constructor

  // Operator overloading
  Vector& operator=(const Vector& v) {}

  Vector operator+(const Vector& v) const {}

  Vector operator-(const Vector& v) const {}

  Vector operator*(double scalar) const {}

  Vector operator/(double scalar) const {}

  double Magnitude() const {}

  Vector Normalize() const {}

//operations (probabilmente non servono più)
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