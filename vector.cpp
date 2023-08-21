#include "vector.hpp"

namespace vc
{
  // Operator overloading
  Vector &Vector::operator=(const Vector &v)
  {
    if (this != &v)
    {
      m_x = v.getX();
      m_y = v.getY();
    }
    return *this;
  }

  Vector Vector::operator+(const Vector &v) const
  {
    return Vector(m_x + v.getX(), m_y + v.getY());
  }

  Vector Vector::operator-(const Vector &v) const
  {
    return Vector(m_x - v.getX(), m_y - v.getY());
  }

  Vector Vector::operator*(double scalar) const
  {
    return Vector(m_x * scalar, m_y * scalar);
  }

  Vector Vector::operator/(double scalar) const
  {
    return Vector(m_x / scalar, m_y / scalar);
  }

  Vector &Vector::operator+=(const Vector &v)
  {
    m_x += v.getX();
    m_y += v.getY();
    return *this;
  }

  bool Vector::operator==(const Vector &v) const
  {
    return m_x == v.getX() && m_y == v.getY();
  }

  // vector functions
  double Vector::Magnitude() const { return std::sqrt(m_x * m_x + m_y * m_y); }

  Vector Vector::normalize() const
  {
    double magnitude = Magnitude();
    if (magnitude > 0)
    {
      return Vector(m_x / magnitude, m_y / magnitude);
    }
    else
    {
      return Vector(m_x, m_y); // cannot normalize a null vector
    }
  }
  // did we even use this?
  double Vector::dotProduct(const Vector &v) const
  {
    double dot = m_x * v.getX() + m_y * v.getY();
    return dot;
  }

  void Vector::set(double a, double b)
  {
    m_x = a;
    m_y = b;
  }

  double Vector::distance(const Vector &v) const
  {
    double dm_x = m_x - v.getX();
    double dm_y = m_y - v.getY();
    double dist = sqrt(dm_x * dm_x + dm_y * dm_y);
    return dist; // its the same of asking the magnitude of the difference vector
  }

  void Vector::limit(double mam_x)
  {
    double magnitude = Magnitude();
    if (magnitude > mam_x)
    {
      *this = normalize() * mam_x;
    }
  }
}