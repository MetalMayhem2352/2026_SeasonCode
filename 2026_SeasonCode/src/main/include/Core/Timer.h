#pragma once

#include <chrono>

namespace Core
{
    using clock = std::chrono::_V2::steady_clock;
    using time_point = std::chrono::_V2::steady_clock::time_point;

    class Timer
    {
    private:
        time_point startTime;
        time_point lastTime;
        float deltaTimeSecconds;

    public:
        Timer();
        ~Timer();

        void Update();
        double GetDeltaTime();
        double GetStartTime();
        void Reset();
    }; 
}
