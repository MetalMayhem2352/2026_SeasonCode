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

	shooterMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Shooter::shooterID, Constants::CANIVOUR_NAME);

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
	shooterTimer.Reset();
}

void Robot::AutonomousPeriodic() 
{
	shooterTimer.Update();

	if (shooterTimer.GetStartTime() > 3 && shooterTimer.GetStartTime() < 4)
	{
		shooterMotor->Set(1);
		intakeModule->UpdateState(intakeModule->Shooting);
	}
	else if (shooterTimer.GetStartTime() > 4)
	{
		shooterTimer.Reset();
	}
	else
	{
		shooterMotor->Set(0);
		intakeModule->UpdateState(intakeModule->Idle);
	}


	intakeModule->Update();	
}

void Robot::TeleopInit() 
{
	swerveDrive->ResetIMU();
}

void Robot::TeleopPeriodic() 
{
	Core::Timer timer{};
	
	/*

	// Left Driver
	// Intake
	if (driver1.GetRawAxis(2) > 0.25)
	{
		if (!isShooting)
		{
			intakeModule->UpdateState(intakeModule->Intaking);
			isIntaking = true;
		}
	
	}
	else if (driver1.GetRawButton(5))
	{
		intakeModule->UpdateState(intakeModule->Outaking);
		isIntaking = true;
	}
	else
	{
		if (isShooting == false)
		{
			intakeModule->UpdateState(intakeModule->Idle);
		}
		isIntaking = false;
	}


	// X
	// Toggle Basket
	if (driver1.GetRawButtonPressed(2))
	{
		if (basketModule->GetState() == basketModule->High)
		{
			basketModule->UpdateState(basketModule->Low);
		}
		else
		{
			basketModule->UpdateState(basketModule->High);
		}
	}
	if (driver1.GetRawButtonPressed(1))
	{
		isTracking = !isTracking;
	}
	
	*/
	

	// Right Trigger
	// Shoot
	if (driver1.GetRawAxis(3) > 0.25)
	{
		if (shooterTimer.GetStartTime() > 3 && shooterTimer.GetStartTime() < 4)
		{
			shooterMotor->Set(1);
			intakeModule->UpdateState(intakeModule->Shooting);
		}
		else if (shooterTimer.GetStartTime() > 4)
		{
			shooterTimer.Reset();
		}
		else
		{
			shooterMotor->Set(0);
			intakeModule->UpdateState(intakeModule->Idle);
		}
	}

	turretModule->Track();


 	turretOffset = driver2.GetRawAxis(4) * 10;
	

	if (driver1.GetRawButtonPressed(7))
	{
		swerveDrive->ResetIMU();
	}

	
	double x = driver1.GetRawAxis(0);
	double z = driver1.GetRawAxis(1);
	double rotation = driver1.GetRawAxis(4);

	swerveDrive->MoveFieldCentric1(x, -z, rotation);
	

	timer.Update();
	std::cout << "Time: " << timer.GetStartTime() << '\n';

	turretModule->Rotate(swerveDrive->GetYaw() - 180 + turretOffset);
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
	turretModule->Rotate(100);
}

void Robot::SimulationInit() 
{

}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  	return frc::StartRobot<Robot>();
}
#endif
