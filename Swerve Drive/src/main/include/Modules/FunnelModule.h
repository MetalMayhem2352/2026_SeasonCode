#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

namespace Modules
{
    class FunnelModule
    {
        private:
            ctre::phoenix6::hardware::TalonFX* funnelMotor;
        public:

            FunnelModule();
            ~FunnelModule();

            void Feed();
            void Unjam();
            void Idle();
        

    };
}