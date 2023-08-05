#ifndef VECTOR_HPP
#define VECTOR_HPP
#include <cmath>

class Vector {
  double x_;
  double y_;

 public:
  Vector(double, double){};

  // Operator overloading
  Vector& operator=(
      const Vector&);  // doesnt need const because it has to modify the
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

  void Set(double, double);

  double distance(const Vector&) const;

  double getX() const { return x_; }

  double getY() const { return y_; }
};

#endif