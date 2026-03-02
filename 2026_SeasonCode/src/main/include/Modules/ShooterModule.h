#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"
#include "Pathing/RobotPosition.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace Modules
{
    class ShooterModule
    {
        private:

            ctre::phoenix6::hardware::TalonFX* shooterMotor;
            ctre::phoenix6::hardware::TalonFX* hoodMotor;

        public:

            ShooterModule();
            ~ShooterModule();

            void Shoot(Pathing::RobotPosition* const position);
            void Stop();
        
    };
}