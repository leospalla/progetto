#include "flock.hpp"
#include <chrono> //libraries used to have a better control over the input/output
#include <thread>

int main() {
  fk::Flock flock;
  int numBoids;
  double time;
  int simulationSteps;
int lenght;

  double perceptionRadius;
  double separationDistance;
  double separationFactor;
  double cohesionFactor;
  double alignmentFactor;

  const int maxNumBoids{1000};
  const double maxSimTime{30.};
  const int maxDimension{1000};
  std::cout << "Simulating the behaviour of boids inside flocks..." << std::endl << std::endl;
 // std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "Please proceed with the input phase (press Ctrl+Z at any time to stop the program)." << std::endl;
  std::cout << "Note: Only numerical inputs are accepted, every non-numerical input won't be considered." << std::endl << std::endl;
 // std::this_thread::sleep_for(std::chrono::seconds(5));


  std::cout << "Insert the number of boids: " << std::endl;
  std::cout << "Note: If you enter a decimal value, only the integer part will be considered." << std::endl;
  
  while (!(std::cin >> numBoids) || numBoids <= 0 || numBoids > maxNumBoids) // checks if the type is correct, if the type is double takes only the integer part
  {
    std::cout << "Invalid input. Please enter a positive integer up to: " << maxNumBoids << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // if there are some wrong characters it ignores them for the next input
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // if u insert a double the number after inteferes so this ignores another time also after the cycle ends

  std::cout << "Insert the time of simulation (seconds): " << std::endl;
  while (!(std::cin >> time) || time <= 0 || time > maxSimTime)
  {
    std::cout << "Invalid input. Please enter a positive double up to: " << maxSimTime << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Insert the lenght of the space: " << std::endl;
  std::cout << "Note: If you enter a decimal value, only the integer part will be considered." << std::endl;
  while (!(std::cin >> lenght) || lenght > maxDimension)
  {
    std::cout << "Invalid input. Please enter a positive integer up to: " << maxDimension << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  
  for (int i = 0; i < numBoids; ++i) {
    bd::Boid b{lenght};
    flock.addBoid(b);
  }

  std::cout << "Insert the perception radius of each boid: " << std::endl;
  while (!(std::cin >> perceptionRadius) || perceptionRadius <= 0 || perceptionRadius >= lenght) {
    std::cout << "Invalid input. Please enter a positive double smaller than the window dimensions." << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Insert the separation distance of the separation rule: " << std::endl;
  while (!(std::cin >> separationDistance) || separationDistance <= 0 || separationDistance >= perceptionRadius) {
    std::cout << "Invalid input. Please enter a positive double smaller than the perception radius." << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Insert the separation factor of the separation rule: " << std::endl;
  while (!(std::cin >> separationFactor) || separationFactor <= 0) {
    std::cout << "Invalid input. Please enter a positive double." << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Insert the cohesion factor of the cohesion rule: " << std::endl;
  while (!(std::cin >> cohesionFactor) || cohesionFactor <= 0) {
    std::cout << "Invalid input. Please enter a positive double." << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Insert the alignment factor of the alignment rule: " << std::endl;
  while (!(std::cin >> alignmentFactor) || alignmentFactor <= 0) {
    std::cout << "Invalid input. Please enter a positive double." << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  flock.updateBoidParameters(perceptionRadius, separationDistance, separationFactor, cohesionFactor, alignmentFactor);

  simulationSteps = time / flock.getDeltaTime();
  flock.simulate(simulationSteps, lenght);

  return 0;
}