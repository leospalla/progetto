#include "vector.hpp"

#include <cmath>
//operations
void Vector::add(const Vector& v) {
    x += v.x;
    y += v.y; 
}

void Vector::multiply(const Vector& v)  {
    x *= v.x;
    y *= v.y;
}

void Vector::subtract(const Vector& v)  {
    x -= v.x;
    y -= v.y;
}

void Vector::mulScalar(double scalar)  {
    x *= scalar;
    y *= scalar;
}

void Vector::divScalar(double scalar)  {
    x /= scalar;
    y /= scalar;
}


double Vector::Magnitude() { 
    return std::sqrt(x * x + y * y); 
    }

void Vector::Normalize() {
  double magnitude = Magnitude();
  x /= magnitude, y /= magnitude;
}

double Vector::dotProduct(const Vector& v) {
  float dot = x * v.x + y * v.y;
  return dot;
}

// manca da aggiungere altre funzioni come set ecc