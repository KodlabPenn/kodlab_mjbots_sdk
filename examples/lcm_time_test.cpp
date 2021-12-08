#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <chrono>
#include "bulk_data.hpp"
#include "kodlab_mjbots_sdk/realtime.h"
#include "real_time_tools/thread.hpp"
using namespace std;

int main()
{
  mjbots::moteus::ConfigureRealtime(1);
  lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");
  if(!lcm.good())
    return 1;
  bulk_data my_data{};
  for(int i = 0; i< 1000; i++){
    my_data.position[i]= i;
  }
  my_data.timestamp = 0;
  const auto now = std::chrono::steady_clock::now();
  lcm.publish("EXAMPLE", &my_data);
  const auto later = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> elapsed = later - now;
  cout <<"Duration =" << elapsed.count() << endl;
  return 0;
}