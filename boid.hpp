#ifndef BOID_HPP
#define BOID_HPP
#include "vector.hpp"
#include <vector>
namespace bd
{
    class Boid
    {
    private:
        Vector position_{};
        Vector velocity_{};
        Vector acceleration_{0., 0.};
        double const maxSpeed_{10.};
        double const ds{15.};

    public:
        bool operator==(const Boid &other) const;
        Boid &operator=(const Boid &other); // it is needed to assign the values of a boid to another boid. I dint use it but it may be usefull.
        double const separationDistance{2.};
        double const separationFactor{1.2};
        double const cohesionFactor{0.8};
        double const alignmentFactor{0.5};
        Boid(Vector, Vector); // Its needed for tests
        Boid(Vector);
        Boid();
        Vector pos() const { return position_; }
        Vector vel() const { return velocity_; }
        Vector acc() const { return acceleration_; }
        double maxSpeed() const { return maxSpeed_; }
        Vector setPosition(double, double);
        Vector setVelocity(double, double);
        Vector centerOfMass(std::vector<Boid>);
        Vector separate(std::vector<Boid>); // I left them in the public because it's easier to test.
        Vector cohere(std::vector<Boid>);
        Vector align(std::vector<Boid>);
        double speed() const;
        void border(unsigned int, unsigned int);
    };
}
#endif