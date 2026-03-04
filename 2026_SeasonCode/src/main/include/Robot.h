// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>

#include "Modules/IntakeModule.h"
#include "Modules/BasketModule.h"
#include "Modules/ShooterModule.h"
#include "Modules/TurretModule.h"
#include "Pathing/SwerveDrive.h"
#include "Modules/TestModule.h"

class Robot : public frc::TimedRobot 
{
 	public:
		Robot();
		~Robot();
		void RobotPeriodic() override;
		void AutonomousInit() override;
		void AutonomousPeriodic() override;
		void TeleopInit() override;
		void TeleopPeriodic() override;
		void DisabledInit() override;
		void DisabledPeriodic() override;
		void TestInit() override;
		void TestPeriodic() override;
		void SimulationInit() override;
		void SimulationPeriodic() override;

 	private:
		frc::SendableChooser<std::string> m_chooser;
		const std::string kAutoNameDefault = "Default";
		const std::string kAutoNameCustom = "My Auto";
		std::string m_autoSelected;
		int varName = 0;

		Modules::IntakeModule* intakeModule;
		Modules::BasketModule* basketModule;
		Modules::ShooterModule* shooterModule;
		Turret_Tracking* turretModule;
		Modules::TestModule* testModule;
		Pathing::SwerveDrive* swerveDrive;

		frc::Joystick bryceController{1};
		frc::Joystick testController{0};
};
