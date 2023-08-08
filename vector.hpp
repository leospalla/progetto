#ifndef VECTOR_HPP
#define VECTOR_HPP

class Vector
{
private:
    double m_x{0.};
    double m_y{0.};

public:
    Vector(double x, double y) : m_x{x}, m_y{y} {}
    Vector(double x) : m_x{x} {}
    Vector() = default;
    double xcomp() const { return m_x; }
    double ycomp() const { return m_y; }
};
double norm(Vector const &v);
double magnitude(Vector const &v);
bool operator==(Vector const &v, Vector const &w);
Vector operator+(Vector const &v, Vector const &w);
Vector operator-(Vector const &v, Vector const &w);
Vector operator*(double c, Vector const &v);
Vector operator/(Vector const &v, double c);
double dotproduct(Vector const &v, Vector const &w);
double distance(Vector const &v, Vector const &w);

#endif