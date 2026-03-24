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

  	swerveDrive = new CustomSwerveDrive::SwerveDriveModule();
  	intakeModule = new Modules::IntakeModule();
  	turretModule = new Turret_Tracking(swerveDrive);
  	shooterModule = new Modules::ShooterModule();
  	funnelModule = new FunnelModule();

	odometery = swerveDrive->CreateSwerveDriveOdometery();

	autoRunner = new Autonomous::AutoRunner(swerveDrive, odometery);
}

Robot::~Robot() 
{
  	delete(intakeModule);
  	delete(turretModule);
  	delete(shooterModule);
  	delete(swerveDrive);
  	delete(funnelModule);
	  
  	delete(autoRunner);
}


void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
	m_autoSelected = m_chooser.GetSelected();
	// m_autoSelected = SmartDashboard::GetString("Auto Selector",
	//     kAutoNameDefault);
	wpi::print("Auto selected: {}\n", m_autoSelected);

  	if (m_autoSelected == "Left Trench") 
	{
    	autoPath = LeftTrench;
  	} 
	else if (m_autoSelected == "Left Bump") 
	{
    	autoPath = LeftBump;
  	} 
	else if (m_autoSelected == "Right Bump") 
	{
    	autoPath = RightBump;
  	} 
	else if (m_autoSelected == "Right Trench") 
	{
    	autoPath = RightTrench;
  	} 

	swerveDrive->ResetIMU();
	shooterTimer.Reset();
	
	switch (autoPath)
	{
		case AutoPath::LeftTrench:
		{
			odometery->SetStartPosition(Constants::Auto::START_POSE__LEFT_TRENCH);

			autoRunner->MakePaths({Constants::Auto::THROUGH_LEFT_TRENCH_POSITION2, 
									Constants::Auto::FAR1_LEFT_INTAKE_POSITION, Constants::Auto::FAR1_RIGHT_INTAKE_POSITION, Constants::Auto::FAR2_RIGHT_INTAKE_POSITION, 
									Constants::Auto::FAR2_LEFT_INTAKE_POSITION, Constants::Auto::FAR3_LEFT_INTAKE_POSITION, Constants::Auto::FAR3_RIGHT_INTAKE_POSITION});
			
			break;
		}
		case AutoPath::LeftBump:
		{
			odometery->SetStartPosition(Constants::Auto::START_POSE__LEFT_BUMP);

			autoRunner->MakePaths({Constants::Auto::THROUGH_LEFT_TRENCH_POSITION1.WithStartupDelay(10), Constants::Auto::THROUGH_LEFT_TRENCH_POSITION2.WithStartupDelay(1), 
									Constants::Auto::FAR1_LEFT_INTAKE_POSITION, Constants::Auto::FAR1_RIGHT_INTAKE_POSITION, Constants::Auto::FAR2_RIGHT_INTAKE_POSITION, 
									Constants::Auto::FAR2_LEFT_INTAKE_POSITION, Constants::Auto::FAR3_LEFT_INTAKE_POSITION, Constants::Auto::FAR3_RIGHT_INTAKE_POSITION});
			
			break;
		}
		case AutoPath::RightBump:
		{
			odometery->SetStartPosition(Constants::Auto::START_POSE__RIGHT_BUMP);

			autoRunner->MakePaths({Constants::Auto::THROUGH_RIGHT_TRENCH_POSITION1.WithStartupDelay(10), Constants::Auto::THROUGH_RIGHT_TRENCH_POSITION2.WithStartupDelay(1), 
									Constants::Auto::FAR1_RIGHT_INTAKE_POSITION.WithReverseHeading(), Constants::Auto::FAR1_LEFT_INTAKE_POSITION.WithReverseHeading(), 
									Constants::Auto::FAR2_LEFT_INTAKE_POSITION.WithReverseHeading(), Constants::Auto::FAR2_RIGHT_INTAKE_POSITION.WithReverseHeading(), 
									Constants::Auto::FAR3_RIGHT_INTAKE_POSITION.WithReverseHeading(), Constants::Auto::FAR3_LEFT_INTAKE_POSITION.WithReverseHeading()});

			break;
		}
		case AutoPath::RightTrench:
		{
			odometery->SetStartPosition(Constants::Auto::START_POSE__RIGHT_TRENCH);

			autoRunner->MakePaths({Constants::Auto::THROUGH_RIGHT_TRENCH_POSITION2, 
									Constants::Auto::FAR1_RIGHT_INTAKE_POSITION.WithReverseHeading(), Constants::Auto::FAR1_LEFT_INTAKE_POSITION.WithReverseHeading(), 
									Constants::Auto::FAR2_LEFT_INTAKE_POSITION.WithReverseHeading(), Constants::Auto::FAR2_RIGHT_INTAKE_POSITION.WithReverseHeading(), 
									Constants::Auto::FAR3_RIGHT_INTAKE_POSITION.WithReverseHeading(), Constants::Auto::FAR3_LEFT_INTAKE_POSITION.WithReverseHeading()});

			break;
		}
	}
	
}

void Robot::AutonomousPeriodic() 
{
	// Constant Shooting

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
		shooterMotor->Set(1);
		intakeModule->UpdateState(intakeModule->Outaking);
	}
	
	intakeModule->Update();	
	// turretModule->Rotate(swerveDrive->GetYaw() - 180 + turretOffset);
	
	// Constant Shooting

}

void Robot::TeleopInit() 
{
}

void Robot::TeleopPeriodic() 
{
	Core::Timer timer{};
	shooterTimer.Update();
	
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

	if (driver1.GetRawAxis(3) > 0.25)
	{
		if (shooterTimer.GetStartTime() < 2.5)
		{
			shooterMotor->Set(1);
			intakeModule->UpdateState(intakeModule->Shooting);
		}
		else if (shooterTimer.GetStartTime() > 3)
		{
			shooterTimer.Reset();
		}
		else
		{
			shooterMotor->Set(1);
			intakeModule->UpdateState(intakeModule->Outaking);
		}
		isShooting = true;
	}
	else if (driver1.GetRawButton(6))
	{
		if (shooterTimer.GetStartTime() < 2.5)
		{
			shooterMotor->Set(1);
		}
		else if (shooterTimer.GetStartTime() > 3)
		{
			shooterTimer.Reset();
		}
		else
		{
			shooterMotor->Set(1);
			intakeModule->UpdateState(intakeModule->Outaking);
		}
		isShooting = true;
	}
	else
	{
		isShooting = false;
		if (isIntaking == false)
		{
			intakeModule->UpdateState(intakeModule->Idle);
		}
		shooterMotor->Set(0);
	}

	// turretModule->Track();

 	turretOffset = driver2.GetRawAxis(4) * 20;
	

	if (driver1.GetRawButtonPressed(7))
	{
		swerveDrive->ResetIMU();
	}

	
	
	

	timer.Update();
	std::cout << "Time: " << timer.GetStartTime() << '\n';

	// turretModule->Rotate(175 - swerveDrive->GetYaw() + turretOffset);
	// turretModule->Rotate(turretOffset);

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
	intakeModule->UpdateState(intakeModule->Intaking);
	funnelModule->Feed();
	shooterModule->ShootAtDistance(0);
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
