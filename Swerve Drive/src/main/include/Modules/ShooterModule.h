#pragma once

#include "Core/PiecewiseLinearFunctionXYZ.h"
#include "Constants.h"

#include <frc/Servo.h>
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
                Pass = 2,
            };
        private:

            frc::Servo hoodServo{9};
            ctre::phoenix6::hardware::TalonFX* shooterMotor;

            State currentState;

            // Distance (meters): Power: HoodAngle
            Core::PiecewiseLinearFunctionXYZ shooingDistanceTable;

        public:

            ShooterModule();
            ~ShooterModule();

            void ShootAtDistance(float distance);
            void Stop();
            void PassBall();

            State GetState();

        private:
            
            void MoveHood(float angle);
        
    };
}