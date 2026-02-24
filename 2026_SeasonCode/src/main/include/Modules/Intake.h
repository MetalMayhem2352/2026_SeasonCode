/*
#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

namespace Modules
{
    class IntakeModule 
    {
    private:
        frc::Timer timer;

        Core::Timer* PIDTimer;
        Core::PIDController* PIDController;

        double tx;

        double forward = 0;
        double backward = 0;
        double currentpos;
        double motorangle;
        double angleoffset;
        double error;

        double maxRotation;
        double minRotation;
        bool hasTarget;

        ctre::phoenix6::hardware::TalonFX turret_motor{Constants::Turret::turretID};

    public:
        enum State
        {
            Intaking = 0,
            Outaking = 1,
            Shooting = 2,
        };  

        IntakeModule();
        ~IntakeModule();

        
    

    };
}
*/