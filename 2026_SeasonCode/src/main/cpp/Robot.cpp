// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>

Robot::Robot() 
{
  	m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  	m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  	intakeModule = new Modules::IntakeModule();
  	basketModule = new Modules::BasketModule();
  	turretModule = new Turret_Tracking();
  	shooterModule = new Modules::ShooterModule();
  	testModule = new Modules::TestModule();
}

Robot::~Robot() 
{
  	delete(intakeModule);
  	delete(basketModule);
  	delete(turretModule);
  	delete(shooterModule);
  	delete(testModule);
}


void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
	m_autoSelected = m_chooser.GetSelected();
	// m_autoSelected = SmartDashboard::GetString("Auto Selector",
	//     kAutoNameDefault);
	wpi::print("Auto selected: {}\n", m_autoSelected);

  	if (m_autoSelected == kAutoNameCustom) 
	{
    	// Custom Auto goes here
  	} 
	else 
	{
    	// Default Auto goes here
  	}
}

void Robot::AutonomousPeriodic() 
{
  	if (m_autoSelected == kAutoNameCustom) 
	{
    	// Custom Auto goes here
  	} 
  	else 
	{
    	// Default Auto goes here
  	}
  
}

void Robot::TeleopInit() 
{
	swerveDrive.~SwerveDrive();
}

void Robot::TeleopPeriodic() 
{
	// Intake
	if (testController.GetRawButtonPressed(-1))
	{
		if (intakeModule->GetState() != Modules::IntakeModule::Idle && shooterModule->GetState() != Modules::ShooterModule::Shoot)
		{
			intakeModule->UpdateState(Modules::IntakeModule::Idle);
		}
		else
		{
			intakeModule->UpdateState(Modules::IntakeModule::Intaking);
		}
	}
	
	// ground outake
	if (testController.GetRawButtonPressed(-1))
	{
		if (intakeModule->GetState() != Modules::IntakeModule::Idle)
		{
			intakeModule->UpdateState(Modules::IntakeModule::Idle);
		}
		else
		{
			intakeModule->UpdateState(Modules::IntakeModule::Outaking);
		}
	}

	// shoot
	if (testController.GetRawButtonPressed(-1))
	{
		if (intakeModule->GetState() == Modules::IntakeModule::Shooting)
		{
			intakeModule->UpdateState(Modules::IntakeModule::Idle);
			shooterModule->Stop();
		}
		else
		{
			intakeModule->UpdateState(Modules::IntakeModule::Shooting);
			shooterModule->ShootAtDistance(turretModule->limelight_Distance());
		}
	}

	// Dpad Up
	if (testController.GetRawButtonPressed(-1))
	{
		basketModule->UpdateState(Modules::BasketModule::High);
	}
	// Dpad Down 
	else if (testController.GetRawButtonPressed(-1))
	{
		basketModule->UpdateState(Modules::BasketModule::Low);
	}

	double x = testController.GetRawAxis(0);
	double y = testController.GetRawAxis(1);
	double rotation = testController.GetRawAxis(4);

	swerveDrive.Move(x,y,rotation);
	

	
}

void Robot::DisabledInit() 
{
	intakeModule->UpdateState(Modules::IntakeModule::Idle);
}

void Robot::DisabledPeriodic() 
{
	turretModule->turretIdle();
	shooterModule->~ShooterModule();
}
void Robot::TestInit() 
{
}

void Robot::TestPeriodic() 
{
	
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  	return frc::StartRobot<Robot>();
}
#endif
