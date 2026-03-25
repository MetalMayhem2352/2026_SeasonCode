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

            TunerSwerveDrivetrain swerveDrive{TunerConstants::DrivetrainConstants, 
                TunerConstants::FrontLeft, TunerConstants::FrontRight, TunerConstants::BackLeft, TunerConstants::BackRight};

            swerve::requests::FieldCentric m_driveRequest = swerve::requests::FieldCentric{}
                .WithDeadband(TunerConstants::kSpeedAt12Volts * 0.1).WithRotationalDeadband(TunerConstants::kMaxAngularSpeed * 0.1) // Add a 10% deadband
                .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
                .WithSteerRequestType(swerve::SteerRequestType::Position);

        public:

            CTRESwerveDrive();
            ~CTRESwerveDrive();

            void Update();
            void Move(double x, double z, double rotation);
            void ResetYaw();
        
    };
}