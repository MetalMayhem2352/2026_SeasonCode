#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

namespace Modules
{
    class Basket
    {
        public:
            enum State
            {
                Low = 0,
                High = 1
            };
        private:
            Core::Timer* pidTimer;
            Core::PIDController* leftPIDController;
            Core::PIDController* rightPIDController;
        
            double targetPosition;

            ctre::phoenix6::hardware::TalonFX* leftBasketMotor;
            ctre::phoenix6::hardware::TalonFX* rightBasketMotor;
        public:
            Basket();
            ~Basket();

            void Update();
            void UpdateState(State newState);
    };
}