#include "vector.hpp"
#include <cmath>

bool operator==(Vector const &v, Vector const &w)
{
    return v.xcomp() == w.xcomp() && v.ycomp() == w.ycomp();
}
Vector operator+(Vector const &v, Vector const &w)
{
    return Vector{v.xcomp() + w.xcomp(), v.ycomp() + w.ycomp()};
}
Vector operator-(Vector const &v, Vector const &w)
{
    return Vector{v.xcomp() - w.xcomp(), v.ycomp() - w.ycomp()};
}
Vector operator*(double c, Vector const &v)
{
    return Vector{c * v.xcomp(), c * v.ycomp()};
}
Vector operator/(Vector const &v, double c)
{
    return Vector{v.xcomp() / c, v.ycomp() / c};
}
double dotproduct(Vector const &v, Vector const &w)

{
    return v.xcomp() * w.xcomp() + v.ycomp() * w.ycomp();
}
double norm(Vector const &v)
{
    double n = dotproduct(v, v);
    return n;
}
double distance(Vector const &v, Vector const &w)
{
    Vector diff = v - w;
    double n = norm(diff);
    return std ::sqrt(n);
}
double magnitude(Vector const &v)
{
    double n = norm(v);
    return std ::sqrt(n);
}
