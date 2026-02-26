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
  	testModule = new Modules::TestModule();
}

Robot::~Robot() 
{
  	delete(intakeModule);
  	delete(basketModule);
  	delete(turretModule);
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

}

void Robot::TeleopPeriodic() 
{
	// Intake
	if (testController.GetRawButtonPressed(-1))
	{
		if (intakeModule->GetState() == Modules::IntakeModule::Intaking)
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
		if (intakeModule->GetState() == Modules::IntakeModule::Outaking)
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
		}
		else
		{
			intakeModule->UpdateState(Modules::IntakeModule::Shooting);
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

	

}

void Robot::DisabledInit() 
{
	testModule->Stop();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() 
{
	testModule->Shoot();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  	return frc::StartRobot<Robot>();
}
#endif
