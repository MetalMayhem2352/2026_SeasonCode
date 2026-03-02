#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

namespace Pathing
{
    class SwerveDrive 
    {
        private:

            ctre::phoenix6::hardware::TalonFX* motor1;

        public:

            SwerveDrive();
            ~SwerveDrive();

            void Update();
            void Move(double x, double z, double rotation);
        
    };
}