#include "Core/Timer.h"


namespace Core
{
    
    Timer::Timer()
    {
        lastTime = clock::now();
        startTime = lastTime;
    }   

    Timer::~Timer()
    {
        
    }

    void Timer::Update()
    {
        time_point currentTime = clock::now();
        std::chrono::duration<double> deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        deltaTimeSecconds = deltaTime.count();
    }
    double Timer::GetDeltaTime()
    {
        return deltaTimeSecconds;
    }
    double Timer::GetStartTime()
    {
        time_point currentTime = clock::now();
        std::chrono::duration<double> deltaTime = currentTime - startTime;
        return deltaTime.count();
    }
    void Timer::Reset()
    {
        startTime = clock::now();
    }
}
