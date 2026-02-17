#include "Core/LidarLitev3Sensor.h"
#include <iostream>

LidarLitev3Sensor::LidarLitev3Sensor(int lidarID)
    : lidar{lidarID}   // Initialize Counter with the PWM port
{
    lidar.SetMaxPeriod(units::time::second_t(1));         // Max measurable period
    lidar.SetSemiPeriodMode(true);  // Measure high-time of pulse
    lidar.Reset();
}

void LidarLitev3Sensor::Update() {
    if (lidar.Get() < 1) {
        distance = 0.0;
    } else {
        // Convert to cm: 10 microseconds per cm
        distance = (lidar.GetPeriod().value() * 1'000'000.0 / 10.0);
    }

    std::cout << "Distance: " << distance << std::endl;
}

double LidarLitev3Sensor::GetDistance() const {
    return distance;
}