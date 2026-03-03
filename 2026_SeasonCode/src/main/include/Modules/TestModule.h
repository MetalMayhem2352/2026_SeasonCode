#pragma once

#include <ctre/phoenix6/TalonFX.hpp>

namespace Modules
{
    class TestModule 
    {
        private:

            ctre::phoenix6::hardware::TalonFX* motor1;

        public:

            TestModule();
            ~TestModule();

            void Shoot();
            void Stop();
        
    };
}