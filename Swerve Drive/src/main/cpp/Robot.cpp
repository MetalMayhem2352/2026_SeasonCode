// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>

Robot::Robot() 
{
    swerveDrive = new Pathing::CTRESwerveDrive();

    funnelModule = new Modules::FunnelModule();
    intakeModule = new Modules::IntakeModule();
    shooterModule = new Modules::ShooterModule();
    turretModule = new Modules::TurretModule();
}

Robot::~Robot() 
{
    delete(swerveDrive);
    
    delete(funnelModule);
    delete(intakeModule);
    delete(shooterModule);
    delete(turretModule);
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
    GabeDrive();
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
    intakeModule->UpdateState(intakeModule->Intaking);
    funnelModule->UpdateState(funnelModule->Feed);
    shooterModule->ShootAtDistance(10);
}   

void Robot::TestExit() {}


void Robot::BryceDrive() 
{
    // RIght Bottom Pattle: Intake Out
    if (driver1.GetPOV(90))
    {
        swerveDrive->ResetYaw();
    }

    if (driver1.GetRawAxis(3)) // Right Trigger ################ Shoot 
    {
        // intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        // TO;DO ADD DISTANCE ESTIMATE ################ ASHER
        shooterModule->ShootAtDistance(0);

        // TO;DO ADD AUTO TRACKING ################ GABE
    }
    else if (driver1.GetRawButton(6)) // Right Bumper ################ Pass
    {
        // intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        shooterModule->PassBall();

        // TO;DO ADD AUTO TRACKING ################ GABE
    }
    else if (driver1.GetRawAxis(2)) // Left Trigger ################ Intake 
    {
        // intakeModule->UpdateState(intakeModule->Intaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else if (driver1.GetRawButton(7)) // Left Bumper ################ Outake
    {
        // intakeModule->UpdateState(intakeModule->Outaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else
    {
        // intakeModule->UpdateState(intakeModule->Idle);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
}

void Robot::GabeDrive() 
{
    // Left Trigger: Intake
    // Left Bumper: Switch
    // Bottom Left Pattle: Outake
    // Right Tirgger: Shoot
    // Right Bumper: Pass
}

void Robot::AsherDrive() 
{
    if (driver1.GetRawButtonPressed(7))
    {
        swerveDrive->ResetYaw();
    }

    if (driver1.GetRawAxis(3)) // Right Trigger ################ Shoot 
    {
        // intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        // TO;DO ADD DISTANCE ESTIMATE ################ ASHER
        shooterModule->ShootAtDistance(0);

        // TO;DO ADD AUTO TRACKING ################ GABE
    }
    else if (driver1.GetRawButton(6)) // Right Bumper ################ Pass
    {
        // intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        shooterModule->PassBall();

        // TO;DO ADD AUTO TRACKING ################ GABE
    }
    else if (driver1.GetRawAxis(2)) // Left Trigger ################ Intake 
    {
        // intakeModule->UpdateState(intakeModule->Intaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else if (driver1.GetRawButton(7)) // Left Bumper ################ Outake
    {
        // intakeModule->UpdateState(intakeModule->Outaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else
    {
        // intakeModule->UpdateState(intakeModule->Idle);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
