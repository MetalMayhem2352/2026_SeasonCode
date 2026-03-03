#pragma once

#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <frc/Timer.h>

namespace Pathing
{
    class SwerveDrive 
    {
        private:

            ctre::phoenix6::hardware::TalonFX* motor1;
            ctre::phoenix6::swerve::SwerveDrivetrain<ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::CANcoder>* swerveDrive;

            /*
            
        swerveDrive = new ctre::phoenix6::swerve::SwerveDrivetrain<ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::CANcoder>
            (TunerConstants::DrivetrainConstants, TunerConstants::FrontLeft, TunerConstants::FrontRight, TunerConstants::BackLeft, TunerConstants::BackRight);
            */

        public:

            SwerveDrive();
            ~SwerveDrive();

            void Update();
            void Move(double x, double z, double rotation);
        
    };
}