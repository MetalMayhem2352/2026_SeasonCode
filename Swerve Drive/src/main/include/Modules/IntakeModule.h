#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>


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
            
            enum PivotState
            {
                IdleMotors = 0,
                Up = 1,
                Down = 2,
                Half = 3,
            };  
        
        private:

            State currentState;
            PivotState currentPivotState;

            ctre::phoenix6::hardware::TalonFX* leftPivotMotor;
            ctre::phoenix6::hardware::TalonFX* rightPivotMotor;

            ctre::phoenix6::hardware::TalonFX* frontIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* backIntakeMotor;
            ctre::phoenix6::hardware::TalonFX* basketMotor;

            Core::Timer* pivotPIDTimer;
            Core::PIDController* pivotPIDController;
            frc::DutyCycleEncoder pivotEncoder{Constants::Intake::PIVOT_ENCODER_ID};

            float targetPivotPosition;

        public:
            

            IntakeModule();
            ~IntakeModule();

            void Update();
            void UpdateState(State newState);
            State GetState();

            void SetPivot(PivotState);
            PivotState GetPivotState();

        private:
        
    };
}
