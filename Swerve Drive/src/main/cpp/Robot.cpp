// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>

Robot::Robot() 
{
  	intakeModule = new Modules::IntakeModule();
  	turretModule = new Modules::TurretModule();
  	shooterModule = new Modules::ShooterModule();
  	funnelModule = new Modules::FunnelModule();
}

Robot::~Robot() 
{   
  	delete(intakeModule);
  	delete(turretModule);
  	delete(shooterModule);
  	delete(swerveDrive);
  	delete(funnelModule);
}

void Robot::RobotPeriodic() {
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    
}

void Robot::TeleopPeriodic() 
{
    // Read joystick (example: left stick for translation, right X for rotation)
    double x = driver1.GetRawAxis(0); // forward
    double y = -driver1.GetRawAxis(1);  // strafe
    double rotation = driver1.GetRawAxis(4); // rotation input

    swerveDrive->Move(x, y, rotation);
    
    if (driver1.GetRawButtonPressed(7))
    {
        swerveDrive->ResetYaw();
    }
}

void Robot::TeleopExit() {}

void Robot::TestInit() 
{

}

void Robot::TestPeriodic() 
{
    std::cout << "hoodServo: " << hoodServo.GetChannel() << "\n";
    hoodServo.Set(1);
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
