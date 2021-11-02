#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "bulk_data.hpp"
using namespace std;

int main()
{
  lcm::LCM lcm;
  if(!lcm.good())
    return 1;
  bulk_data my_data{};
  for(int i = 0; i< 100; i++){
    my_data.position[i]= i;
  }
  my_data.timestamp = 0;
  lcm.publish("EXAMPLE", &my_data);
  cout << "Hello, World!";
  return 0;
}