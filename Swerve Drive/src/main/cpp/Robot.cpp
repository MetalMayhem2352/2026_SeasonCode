// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

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

    swerveDrive.SetControl(m_driveRequest.WithVelocityX(y * TunerConstants::kSpeedAt12Volts).WithVelocityY(x * TunerConstants::kSpeedAt12Volts).WithRotationalRate(rotation * TunerConstants::kRotationSpeedAt12Volts));

    if (driver1.GetRawButtonPressed(7))
    {
        std::cout << "1\n";
        swerveDrive.SeedFieldCentric();
    }
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
        
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
