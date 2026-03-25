#pragma once

#include <frc/Counter.h>

class LidarLitev3Sensor {
public:
    explicit LidarLitev3Sensor(int lidarID);

    void Update();
    double GetDistance() const;

private:
    frc::Counter lidar;
    double distance = 0.0;
};