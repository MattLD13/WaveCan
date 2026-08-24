// Diagnostic client for the external grayson-arendt/sparkcan library.
// sparkcan is MIT-licensed and is not vendored into this repository.

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "SparkMax.hpp"

int main()
{
  try
  {
    SparkMax motor("can1", 1);
    motor.SetPeriodicStatus1Period(20);
    motor.SetPeriodicStatus2Period(20);

    auto start = std::chrono::steady_clock::now();
    auto end = start + std::chrono::seconds(5);
    auto next_print = start;

    while (std::chrono::steady_clock::now() < end)
    {
      motor.Heartbeat();
      motor.SetDutyCycle(0.10f);
      auto now = std::chrono::steady_clock::now();
      if (now >= next_print)
      {
        std::cout << std::fixed << std::setprecision(3)
                  << "vel=" << motor.GetVelocity()
                  << " pos=" << motor.GetPosition()
                  << " duty=" << motor.GetDutyCycle()
                  << std::endl;
        next_print = now + std::chrono::milliseconds(100);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    for (int i = 0; i < 5; ++i)
    {
      motor.Heartbeat();
      motor.SetDutyCycle(0.0f);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "done" << std::endl;
    return 0;
  }
  catch (const std::exception& ex)
  {
    std::cerr << "probe failed: " << ex.what() << std::endl;
    return 1;
  }
}
