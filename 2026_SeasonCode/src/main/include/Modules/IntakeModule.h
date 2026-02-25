#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

namespace Modules
{
    class IntakeModule 
    {
    private:
        ctre::phoenix6::hardware::TalonFX* topIntakeMotor;
        ctre::phoenix6::hardware::TalonFX* basketIntakeMotor;
        ctre::phoenix6::hardware::TalonFX* groundIntakeMotor;

    public:
        enum State
        {
            Intaking = 0,
            Outaking = 1,
            Shooting = 2,
        };  

        IntakeModule();
        ~IntakeModule();

        void Update();
        void UpdateState(State newState);
    };
}
