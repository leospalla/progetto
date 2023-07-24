#include "vector.hpp"

#include <cmath>

// Operator overloading
Vector& Vector::operator=(const Vector& v) {
  if (this != &v) {
    x = v.x;
    y = v.y;
  }
  return *this;
}

Vector Vector::operator+(const Vector& v) const {
  return Vector(x + v.x, y + v.y);
}

Vector Vector::operator-(const Vector& v) const {
  return Vector(x - v.x, y - v.y);
}

Vector Vector::operator*(double scalar) const {
  return Vector(x * scalar, y * scalar);
}

Vector Vector::operator/(double scalar) const {
  return Vector(x / scalar, y / scalar);
}

//vector functions
double Vector::Magnitude() const { return std::sqrt(x * x + y * y); }

Vector Vector::Normalize() const {
  double magnitude = Magnitude();
  return Vector(x / magnitude, y / magnitude);
}

double Vector::dotProduct(const Vector& v) const {
  double dot = x * v.x + y * v.y;
  return dot;
}

// manca da aggiungere altre funzioni come set ecc