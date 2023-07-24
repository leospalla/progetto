#ifndef VECTOR_HPP
#define VECTOR_HPP
#include <cmath>

class Vector {
 public:
  double x;
  double y;

  Vector(double xp, double yp) : x{xp}, y{yp} {}  // constructor

  // Operator overloading
  Vector& operator=(const Vector&); // doesnt need const because it has to modify the
                                    // internal state of the object, instead since the other
                                    // operators or functions shouldn't modify the internal
                                    // state of the object, const is needed

  Vector operator+(const Vector&) const;

  Vector operator-(const Vector&) const;

  Vector operator*(double) const;

  Vector operator/(double) const;

  double Magnitude() const;

  Vector Normalize() const;

  double dotProduct(const Vector&) const;
};

#endif