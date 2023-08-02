#ifndef BOID_HPP
#define BOID_HPP
#include "vector.hpp"
#include <vector>
class Boid
{
    private:
    Vector position_{};
    Vector velocity_{};
    public:
    Boid(Vector r, Vector v) : position_{r}, velocity_{v} {}
    Boid(Vector r) : position_{r} {}
    Boid() = default;
    Vector pos() const {return position_;}
    Vector vel() const {return velocity_;} 
};
void separation();
void allignment();
void cohesion();
#endif