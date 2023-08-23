#include "flock.hpp"
int main()
{
  fk::Flock flock;
  
  double perceptionRadius;
  double separationDistance;
  double separationFactor;
  double cohesionFactor;
  double alignmentFactor;
  
  int numBoids;
  double time;
  int simulationSteps;
  unsigned int windowWidth;
  unsigned int windowHeight;

  const int maxNumBoids{500};
  const double maxSimTime{30.};

  std::cout << "Insert the number of boids: " << std::endl;
  while (!(std::cin >> numBoids) || numBoids <= 0 || numBoids > maxNumBoids) // checks if the type is correct, if the type is double takes only the integer part
  {
    std::cout << "Invalid input. Please enter a positive integer or reduce the value: " << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // if there are some wrong characters it ignores them for the next input
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // if u insert a double the number after inteferes so this ignores another time also after the cycle ends

  for (int i = 0; i < numBoids; ++i)
  {
    bd::Boid b;
    flock.addBoid(b);
  }

  std::cout << "Insert the time of simulation (seconds): " << std::endl;
  while (!(std::cin >> time) || time <= 0 || time > maxSimTime)
  {
    std::cout << "Invalid input. Please enter a positive double or reduce the value: " << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::cout << "Insert the window width : " << std::endl;
  while (!(std::cin >> windowWidth) || windowWidth <= 0)
  {
    std::cout << "Invalid input. Please enter a positive integer: " << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::cout << "Insert the window height : " << std::endl;
  while (!(std::cin >> windowHeight) || windowHeight <= 0)
  {
    std::cout << "Invalid input. Please enter a positive integer: " << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Insert the perception radius of each boid: " << std::endl;
  std::cin >> perceptionRadius;
  std::cout << "Insert the separation distance of the separation rule: " << std::endl;
  std::cin >> separationDistance;
  std::cout << "Insert the separation factor of the separation rule: " << std::endl;
  std::cin >> separationFactor;
  std::cout << "Insert the cohesion factor of the cohesion rule: " << std::endl;
  std::cin >> cohesionFactor;
  std::cout << "Insert the alignment factor of the alignment rule: " << std::endl;
  std::cin >> alignmentFactor;

  flock.updateBoidParameters(perceptionRadius, separationDistance, separationFactor, cohesionFactor, alignmentFactor);
  simulationSteps = time / flock.getDeltaTime();
  flock.simulate(simulationSteps, windowWidth, windowHeight);
  return 0;
}