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
            double intakePivotPos;
            double targetPivotPos;
            enum State
            {
                Idle = 0,
                Intaking = 1,
                Shooting = 2,
                Outaking = 3,
                Unjamming = 4,
                GroundShoot = 5,
                intake_up = 6
            };  
        private:
            double targetPivotPos;

            State currentState;

            Core::Timer* pivotPIDTimer;
            Core::PIDController* pivotPIDController;

            ctre::phoenix6::hardware::TalonFX* frontIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* basketIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* bottomIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* intakePivot;
            ctre::phoenix6::hardware::TalonFX* intakePivot2;

        public:
            

            IntakeModule();
            ~IntakeModule();

            void Update();
            void UpdateState(State newState);
            State GetState();
    };
}
