#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"
#include "Pathing/TunerConstants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <frc/Timer.h>

namespace Pathing
{
    class CTRESwerveDrive 
    {
        private:

            ctre::phoenix6::swerve::SwerveDrivetrain<ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::CANcoder> 
                swerveDrive{TunerConstants::DrivetrainConstants, TunerConstants::FrontLeft, TunerConstants::FrontRight, TunerConstants::BackRight, TunerConstants::BackLeft};
                
        public:

            CTRESwerveDrive();
            ~CTRESwerveDrive();

            void Update();
            void Move(double x, double z, double rotation);
        
    };
}