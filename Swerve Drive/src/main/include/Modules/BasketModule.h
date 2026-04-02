#pragma once

#include "Core/Timer.h"
#include "Constants.h"

#include <frc/Servo.h>

namespace Modules
{
    class BasketModule
    {
        public:
            enum State
            {
                Up = 0,
                Down = 1,
                Sweep = 2,
            };
        private:

            State currentState;
            
            frc::Servo hoodServo{0};
            frc::Servo right{8};
        public:
            

            BasketModule();
            ~BasketModule();

            void UpdateState(State newState);
            State GetState();
            
    };
}
