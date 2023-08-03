#ifndef VECTOR_HPP
#define VECTOR_HPP
#include <cmath>

class Vector {


 public:
  double x;
  double y;  
  Vector() : x{0.}, y{0.} {}                      // default constructor
  Vector(double xp, double yp) : x{xp}, y{yp} {}  // other constructor
  Vector(const Vector& other) : x{other.x}, y{other.y} {} //copy constructor
  // Operator overloading
  Vector& operator=(const Vector&);  // doesnt need const because it has to modify the
                                     // internal state of the object, instead since the other
                                     // operators or functions shouldn't modify the internal
                                     // state of the object, const is needed

  Vector operator+(const Vector&) const;

  Vector operator-(const Vector&) const;

  Vector operator*(double) const;

  Vector operator/(double) const;

  Vector& operator+=(const Vector&);

  double Magnitude() const;

  Vector Normalize() const;

  double dotProduct(const Vector&) const;

  void Set(double, double);

  double distance(const Vector&) const;

  double angle(const Vector&) const;

  void limit(double);
};

#endif