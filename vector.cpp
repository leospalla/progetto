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

double Vector::Magnitude() const { return std::sqrt(x * x + y * y); }

Vector Vector::Normalize() const {
  double magnitude = Magnitude();
  return Vector(x / magnitude, y / magnitude);
}

// operations
void Vector::add(const Vector& v) {
  x += v.x;
  y += v.y;
}

void Vector::multiply(const Vector& v) {
  x *= v.x;
  y *= v.y;
}

void Vector::subtract(const Vector& v) {
  x -= v.x;
  y -= v.y;
}

void Vector::mulScalar(double scalar) {
  x *= scalar;
  y *= scalar;
}

void Vector::divScalar(double scalar) {
  x /= scalar;
  y /= scalar;
}

double Vector::Magnitude() { return std::sqrt(x * x + y * y); }

void Vector::Normalize() {
  double magnitude = Magnitude();
  x /= magnitude, y /= magnitude;
}

double Vector::dotProduct(const Vector& v) {
  float dot = x * v.x + y * v.y;
  return dot;
}

// manca da aggiungere altre funzioni come set ecc