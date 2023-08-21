#ifndef VECTOR_HPP
#define VECTOR_HPP
#include <cmath>

class Vector {
  double m_x{0.};
  double m_y{0.};

 public:
  Vector() = default;                             // default constructor
  Vector(double x, double y) : m_x{x}, m_y{y} {}  // other constructor
  Vector(const Vector& other)
      : m_x{other.getX()}, m_y{other.getY()} {}  // copy constructor

  // Operator overloading
  Vector& operator=(const Vector&);

  Vector operator+(const Vector&) const;

  Vector operator-(const Vector&) const;

  Vector operator*(double) const;

  Vector operator/(double) const;

  Vector& operator+=(const Vector&);

  bool operator==(const Vector&) const;

  double getX() const { return m_x; }

  double getY() const { return m_y; }

  double Magnitude() const;

  Vector normalize() const;

  double dotProduct(const Vector&) const;

  void set(double, double);

  double distance(const Vector&) const;

  void limit(double);
};

#endif