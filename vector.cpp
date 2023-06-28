#include <vector.hpp>
#include <cmath>

class Vector {
    double m_x;
    double m_y;

public:

    Vector(double x, double y) : m_x(x), m_y(y) {}

    // Operator overloading
    Vector operator+(const Vector& v) const {
        return Vector(m_x + v.m_x, m_y + v.m_y);
    }

    Vector operator-(const Vector& v) const {
        return Vector(m_x - v.m_x, m_y - v.m_y);
    }

    Vector operator*(float scalar) const {
        return Vector(m_x * scalar, m_y * scalar);
    }

    Vector operator/(float scalar) const {
        return Vector(m_x / scalar, m_y / scalar);
    }

    float Magnitude() const {
        return std::sqrt(m_x * m_x + m_y * m_y);
    }

    Vector Normalize() const {
        float magnitude = Magnitude();
        return Vector(m_x / magnitude, m_y / magnitude);
    }
};

//mancano da aggiungere altre funzioni come set, dot product ecc