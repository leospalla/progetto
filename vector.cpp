#include "vector.hpp"

#include <cmath>
#include <iostream>
// Operator overloading
Vector& Vector::operator=(const Vector& v) {
  if (this != &v) {
    x_ = v.getX();
    y_ = v.getY();
  }
  return *this;
}

Vector Vector::operator+(const Vector& v) const {
  return Vector(x_ + v.getX(), y_ + v.getY());
}

Vector Vector::operator-(const Vector& v) const {
  return Vector(x_ - v.getX(), y_ - v.getY());
}

Vector Vector::operator*(double scalar) const {
  return Vector(x_ * scalar, y_ * scalar);
}

Vector Vector::operator/(double scalar) const {
  return Vector(x_ / scalar, y_ / scalar);
}

// vector functions
double Vector::Magnitude() const { return std::sqrt(x_ * x_ + y_ * y_); }

Vector Vector::Normalize() const {
  double magnitude = Magnitude();
  if (magnitude > 0) {
    return Vector(x_ / magnitude, y_ / magnitude);
  } else {
    return Vector(x_, y_);  // cannot normalize a null vector
  }
}

double Vector::dotProduct(const Vector& v) const {
  double dot = x_ * v.getX() + y_ * v.getY();
  return dot;
}

void Vector::Set(double a, double b) {
  x_ = a;
  y_ = b;
}

double Vector::distance(const Vector& v) const {
  double dx = x_ - v.getX();
  double dy = y_ - v.getY();
  double dist = sqrt(dx * dx + dy * dy);
  return dist;  // its the same of asking the magnitude of the difference vector
}