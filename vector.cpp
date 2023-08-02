#include "vector.hpp"
#include <cmath>

bool Vector ::operator==(Vector const &v, Vector const &w) const
{
    return v.xcomp() == w.xcomp() && v.ycomp() == w.ycomp();
}
Vector Vector ::operator+(Vector const &v, Vector const &w) const
{
    return Vector{v.xcomp() + w.xcomp(), v.ycomp() + w.ycomp()};
}
Vector Vector ::operator-(Vector const &v, Vector const &w) const
{
    return Vector{v.xcomp() - w.xcomp(), v.ycomp() - w.ycomp()};
}
Vector Vector ::operator*(double c, Vector const &v) const
{
    return Vector{c * v.xcomp(), c * v.ycomp()};
}
Vector Vector ::operator/(double c, Vector const &v) const
{
    return Vector{v.xcomp() / c, v.ycomp() / c};
}
double Vector ::dotproduct(Vector const &v, Vector const &w) const

{
    return v.xcomp() * w.xcomp() + v.ycomp() * w.ycomp();
}
double Vector ::norm(Vector const &v) const
{
    double n = dotproduct(v, v);
    return n;
}
double Vector ::distance(Vector const &v, Vector const &w) const
{
    Vector diff = v - w;
    double n = norm(diff);
    return std ::sqrt(n);
}
