#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

namespace Modules
{
    class TestModule 
    {
    private:

        ctre::phoenix6::hardware::TalonFX* flywheelMotor1;
        ctre::phoenix6::hardware::TalonFX* flywheelMotor2;

    public:
        enum State
        {
            Intaking = 0,
            Outaking = 1,
            Shooting = 2,
        };  

        TestModule();
        ~TestModule();

        void Shoot();

        void Stop();
        
    };
}