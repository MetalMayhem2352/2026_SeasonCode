// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <optional>
#include <frc/Joystick.h>

#include "Pathing/CTRESwerveDrive.h"

#include "Modules/FunnelModule.h"
#include "Modules/IntakeModule.h"
#include "Modules/ShooterModule.h"
#include "Modules/TurretModule.h"

class Robot : public frc::TimedRobot {
public:
    Robot();
    ~Robot();

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
    frc::Servo hoodServo{9};

    Pathing::CTRESwerveDrive* swerveDrive;
    Modules::FunnelModule* funnelModule;
    Modules::IntakeModule* intakeModule;
    Modules::ShooterModule* shooterModule;
    Modules::TurretModule* turretModule;
        
    frc::Joystick driver1{0};
};
