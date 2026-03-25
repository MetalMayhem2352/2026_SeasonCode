#pragma once

#include "PIDConfig.h"

namespace Core
{
    class PIDController
    {
        private:
            PIDConfig config;

            double errorSum = 0;
            
            double lastError;

            bool canLoop = false;
            double loopMin;
            double loopMax;
        
        public:
            PIDController(PIDConfig config);
            ~PIDController();

            void SetLoop(bool canLoop, double loopMin, double loopMax);
            void Reset();
            double Calculate(double currentPosition, double targetPosition, double deltaTime);
    };
}