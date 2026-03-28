#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"
#include "Pathing/RobotPosition.h"

#include <frc/Servo.h>
#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace Modules
{
    class ShooterModule
    {
        public:
            enum State
            {
                Idle = 0,
                Shoot = 1,
            };
        private:

            ctre::phoenix6::hardware::TalonFX* shooterMotor;

            State currentState;

        public:

            ShooterModule();
            ~ShooterModule();

            void ShootAtDistance(double distance);
            void Stop();

            State GetState();
        
    };
}