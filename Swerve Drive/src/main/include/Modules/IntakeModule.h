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
            };  
        
        private:

            State currentState;

            ctre::phoenix6::hardware::TalonFX* leftPivotMotor;
            ctre::phoenix6::hardware::TalonFX* rightPivotMotor;

            ctre::phoenix6::hardware::TalonFX* frontIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* backIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* basketMotor;

        public:
            

            IntakeModule();
            ~IntakeModule();

            void Update();
            void UpdateState(State newState);
            State GetState();
    };
}
