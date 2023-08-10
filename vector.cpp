#include "vector.hpp"

// Operator overloading
Vector& Vector::operator=(const Vector& v) {
  if (this != &v) {
    x = v.x;
    y = v.y;
  }
  return *this;
}

// Remember that symmetric operators could have been implemented as @= operators
// with const references

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

Vector& Vector::operator+=(const Vector& v) {
  x += v.x;
  y += v.y;
  return *this;
}

/* bool Vector::operator==(const Vector& v) {
  return x == v.x && y == v.y; 
} */

// vector functions
double Vector::Magnitude() const { return std::sqrt(x * x + y * y); }

Vector Vector::Normalize() const {
  double magnitude = Magnitude();
  if (magnitude > 0) {
    return Vector(x / magnitude, y / magnitude);
  } else {
    return Vector(x, y);  // cannot normalize a null vector
  }
}

double Vector::dotProduct(const Vector& v) const {
  double dot = x * v.x + y * v.y;
  return dot;
}

void Vector::Set(double a, double b) {
  x = a;
  y = b;
}

double Vector::distance(const Vector& v) const {
  double dx = x - v.x;
  double dy = y - v.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist;  // its the same of asking the magnitude of the difference vector
}

double Vector::angle(const Vector& v) const {
  if (x == 0 && y == 0) {
    return 0.;
  }
  if (v.x == 0 && v.y == 0) {
    return 0.;
  }
  double angleRad =
      std::atan2(v.y, v.x) -
      std::atan2(
          y, x);  // atan2 is a sl fucntion that computes the arc tangent of y/x
  double angleDeg = angleRad * 180.0 / M_PI;
  if (angleDeg < 0) {
    angleDeg += 360.0;
  }

  return angleDeg;
}

void Vector::limit(double max) {
  double magnitude = Magnitude();
  if (magnitude > max) {
    *this = Normalize() * max;
  }
}