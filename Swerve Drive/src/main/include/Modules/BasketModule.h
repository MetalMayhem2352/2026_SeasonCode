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
            };
        private:

            State currentState;
            
            frc::Servo left{Constants::Basket::LEFT_SERVO_ID};
            frc::Servo right{Constants::Basket::RIGHT_SERVO_ID};
        public:
            
            BasketModule();
            ~BasketModule();

            void UpdateState(State newState);
            State GetState();

        private:
            void Move(double posiiton);
            
    };
}
