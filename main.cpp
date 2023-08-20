#include "flock.hpp"
int main()
{
  fk::Flock flock;
  int numBoids;
  std::cout << "Insert the number of boids: " << std::endl;
  while (!(std::cin >> numBoids) || numBoids <= 0) //checks if the type is correct, if the type is double takes only the integer part
  {
    std::cout << "Invalid input. Please enter a positive integer: ";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // if there are some wrong characters it ignorsed them for the next input
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // if u insert a double the number after inteferes so this ignores another time also after the cycle ends
  for (int i = 0; i <= numBoids; ++i)
  {
    bd::Boid b;
    flock.addBoid(b);
  }
  double time;
  int simulationSteps;
  unsigned int windowWidth;
  unsigned int windowHeight;
  std::cout << "Insert the time (seconds): " << std::endl;
  while (!(std::cin >> time) || time <= 0)
  {
    std::cout << "Invalid input. Please enter a positive double: ";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::cout << "Insert the window width : " << std::endl;
   while (!(std::cin >> windowWidth) || windowWidth <= 0)
  {
    std::cout << "Invalid input. Please enter a positive integer: ";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::cout << "Insert the window height : " << std::endl;
   while (!(std::cin >> windowHeight) || windowHeight <= 0)
  {
    std::cout << "Invalid input. Please enter a positive integer: ";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  simulationSteps = time / flock.getDeltaTime();
  flock.simulate(simulationSteps, windowWidth, windowHeight);
  return 0;
}