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
  	swerveDrive = new CustomSwerveDrive::SwerveDriveModule();
}

Robot::~Robot() 
{
  	delete(intakeModule);
  	delete(basketModule);
  	delete(turretModule);
  	delete(shooterModule);
  	delete(swerveDrive);
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

	
	int x = driver1.GetRawAxis(0);
	int z = driver1.GetRawAxis(1);
	int rotation = driver1.GetRawAxis(4);

	swerveDrive->MoveFieldCentric1(x, z, rotation);
	
	intakeModule->Update();	
}

void Robot::DisabledInit() 
{
}

void Robot::DisabledPeriodic() 
{

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
