#include "Core/PIDController.h"
#include "math.h"
#include <iostream>

namespace Core
{
    PIDController::PIDController(PIDConfig config)
        : config(config.kP, config.maxError, config.kI, config.kIActiveZone, config.kD)
    {
        
    }

    PIDController::~PIDController()
    {

    }

    void PIDController::SetLoop(bool canLoop, double loopMin, double loopMax)
    {
        this->canLoop = canLoop;
        this->loopMin = loopMin;
        this->loopMax = loopMax;
    }
    void PIDController::Reset()
    {
        errorSum = 0;
    }
    double PIDController::Calculate(double currentPosition, double targetPosition, double deltaTime)
    {
        double difference = loopMax - loopMin;
        double error = 0;
        double errorRate = 0;

        if (canLoop)
        {
            while (targetPosition > loopMax)
            {
                targetPosition -= difference;
            }
            while (targetPosition < loopMin)
            {
                targetPosition += difference;
            }
            
            error = targetPosition - currentPosition;

            while (error > loopMax)
            {
                error -= difference;
            }
            while (error < loopMin)
            {
                error += difference;
            }
         
            double directDifference = std::abs(error);
            double loopDifference = difference - std::abs(error);

            if (directDifference < loopDifference)
            {
                error = error;
            }
            else
            {
                // Loop power is faster
                error = -difference + error;
            }
        }
        else
        {
            error = targetPosition - currentPosition;
        }
        

        if (config.maxError > 0)
        {
            if (error > config.maxError)
            {
                error = config.maxError;
            }
        }

        // Proportional
        if (config.kP != 0)
        {
            if (config.maxError >= 0 && std::abs(error) > config.maxError)
            {
                error = config.maxError * (error / std::abs(error)); // making it negitive if it is positive
            }
        }

        // Interval
        if (config.kI != 0)
        {
            if (config.kIActiveZone >= 0)
            {   
                if (std::abs(error) < config.kIActiveZone)
                {
                    errorSum += error * deltaTime;
                    // If passes the target then set error sum to 0
                    if ((error < 0 && lastError > 0) || (error > 0 && lastError < 0))
                    {
                        errorSum = 0;
                    }
                }
                else
                {
                    errorSum = 0;
                }
            }
            else
            {
                errorSum += error * deltaTime;
            }
        }

        // Derivative
        if (config.kD != 0)
        {
            if (std::abs(error) < config.maxError)
            {
                // std::cout << "error: " << error << "; Last error: " << lastError << "; Error Rate: " << (error - lastError) << '\n';
                if (error != lastError)
                {
                    errorRate = (error - lastError) / deltaTime;
                }
            }
        }

        lastError = error;
        return ((config.kP * error) + (config.kI * errorSum) + (config.kD * errorRate)) / config.maxError;
    }

}
