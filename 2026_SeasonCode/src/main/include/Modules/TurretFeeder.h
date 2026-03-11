#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

class TurretFeeder
{
    private:
        ctre::phoenix6::hardware::TalonFX* feederMotor;
        

    public:
        void Feed();
        void Unjuam();
        void Idle();
        ~TurretFeeder();
        

};