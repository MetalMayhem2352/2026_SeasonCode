// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <optional>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Pathing/CTRESwerveDrive.h"

#include "Modules/FunnelModule.h"
#include "Modules/IntakeModule.h"
#include "Modules/ShooterModule.h"
#include "Modules/NewTurretModule.h"
#include "Modules/NetworkTableModule.h"
#include "Modules/BasketModule.h"


class Robot : public frc::TimedRobot 
{
    private:
        Pathing::CTRESwerveDrive* swerveDrive;
        Pathing::Odometry* odometry;

        Modules::FunnelModule* funnelModule;
        Modules::IntakeModule* intakeModule;
        Modules::ShooterModule* shooterModule;
        // Modules::NewTurretModule* turretModule;
        Modules::BasketModule* basketModule;

        Modules::NetworkTableModule* networkTableModule;

        frc::Joystick driver1{0};
        frc::Joystick driver2{1};
        bool intakePivotToggle = true;
        bool intakePivotTogglePressed = false;

        double goalDistance;
        double goalAngle;

        Core::Timer* timer;
        bool isPreparingShooting;
        bool isShooting;
        
        bool armUp = true;
        

        Core::Timer* autoTimer;

        double targetTurretAngle;

        double shooterPower = 0.6;

        double x;
        double y;
        double rotation;



        
        frc::SendableChooser<std::string> m_chooser;
        const std::string kAutoNameDefault = "Default";
        const std::string kAutoNameTrench = "Trench";
        const std::string kAutoNameBump = "Bump";
        const std::string kAutoNameHub = "Hub";
        std::string m_autoSelected;

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

        void BryceDrive();
        void GabeDrive();
        void AsherDrive();
        void SeccondDriveAim();

        void Test2();


        void BumpAuto();
        void TrenchAuto();
        void HubAuto();
};
