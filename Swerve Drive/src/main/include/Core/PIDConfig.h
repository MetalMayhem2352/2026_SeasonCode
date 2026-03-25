#pragma once

namespace Core
{
    struct PIDConfig
    {
        public:
            const float kP;
            const float maxError;
            const float kI;
            const float kIActiveZone;
            const float kD;

            PIDConfig(float kP, float maxError, float kI, float kIActiveZone, float kD)
                : kP(kP), maxError(maxError), kI(kI), kIActiveZone(kIActiveZone), kD(kD)
            {

            }
    };   
}