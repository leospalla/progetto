#include "flocktelly.hpp"
int main() //this was just to try
{
    Flock flock;
    flock.addBoid(Boid(Vector(1., 0.))); // would be cool to add a cycle
    flock.addBoid(Boid(Vector(0., 1.)));
    flock.addBoid(Boid(Vector(1., 1.)));
    int simulationSteps = 10;
    flock.simulate(simulationSteps);
    return 0;
}