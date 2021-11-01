#include <iostream>
#include <lcm/lcm-cpp.hpp>
using namespace std;

int main()
{
  lcm::LCM lcm;
  if(!lcm.good())
    return 1;
  cout << "Hello, World!";
  return 0;
}