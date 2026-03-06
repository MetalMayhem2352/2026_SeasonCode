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
        public:
            enum State
            {
                Idle = 0,
                Intaking = 1,
                Shooting = 2,
                Outaking = 3,
                Unjamming = 4,
            };  
        private:
            double targetPivotPos;

            State currentState;

            Core::Timer* pivotPIDTimer;
            // Core::PIDController* pivotPIDController;

            ctre::phoenix6::hardware::TalonFX* topIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* basketIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* groundIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* intakePivot;

        public:
            

            IntakeModule();
            ~IntakeModule();

            void Update();
            void UpdateState(State newState);
            State GetState();
    };
}
