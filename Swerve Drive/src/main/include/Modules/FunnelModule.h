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
        public:
            enum State
            {
                Idle = 0,
                Feed = 1,
                Unjam = 2
            }; 

        private:
            State currentState;

            ctre::phoenix6::hardware::TalonFX* funnelMotor;
        
        public:

            FunnelModule();
            ~FunnelModule();

            void Update();
            void UpdateState(State state);

            State GetState();
        

    };
}