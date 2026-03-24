// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <optional>
#include <frc/Joystick.h>

#include "RobotContainer.h"

#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

class Robot : public frc::TimedRobot {
public:
    Robot();
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void DisabledExit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestInit() override;
    void TestPeriodic() override;
    void TestExit() override;

private:

    TunerSwerveDrivetrain swerveDrive{TunerConstants::DrivetrainConstants, 
        TunerConstants::FrontLeft, TunerConstants::FrontRight, TunerConstants::BackLeft, TunerConstants::BackRight};

    swerve::requests::FieldCentric m_driveRequest = swerve::requests::FieldCentric{}
        .WithDeadband(TunerConstants::kSpeedAt12Volts * 0.1).WithRotationalDeadband(TunerConstants::kRotationSpeedAt12Volts * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
        .WithSteerRequestType(swerve::SteerRequestType::Position);
        
    frc::Joystick driver1{0};
};
