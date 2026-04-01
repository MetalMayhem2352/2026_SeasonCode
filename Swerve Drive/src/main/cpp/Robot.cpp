// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>

Robot::Robot() 
{
    swerveDrive = new Pathing::CTRESwerveDrive();
    odometry = swerveDrive->GetOdometery();

    funnelModule = new Modules::FunnelModule();
    intakeModule = new Modules::IntakeModule();
    shooterModule = new Modules::ShooterModule();
    turretModule = new Modules::NewTurretModule();

    networkTableModule = new Modules::NetworkTableModule();

    timer = new Core::Timer();
}


Robot::~Robot() 
{
    delete(swerveDrive);
    
    delete(funnelModule);
    delete(intakeModule);
    delete(shooterModule);
    delete(turretModule);

    delete(networkTableModule);
}

int i = 0;
void Robot::RobotPeriodic() 
{
    i++;

    if (i % 50 == 0)
    {
        std::cout << "goalDistance: " << (goalDistance) <<'\n';
    }

    networkTableModule->Update();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{
    
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() 
{
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    intakePivotTogglePressed = false;
    intakePivotToggle = false;
}

void Robot::TeleopPeriodic() 
{
    frc::Pose2d currentPosition = odometry->GetPose();
    double headingInRadians = currentPosition.Rotation().Radians().value();
    // shooter is 9" back form the center. Use trig to add that 9" 
    // frc::Pose2d shooerPosition(currentPosition.X() - (9_in * std::cos(headingInRadians)), currentPosition.Y() - (9_in * std::sin(headingInRadians)), currentPosition.Rotation());
    
    goalDistance = std::sqrt(std::pow(Constants::goalPosition.X().value() - currentPosition.X().value(), 2) + std::pow(Constants::goalPosition.Y().value() - currentPosition.Y().value(), 2)) * 1000 / 25.4;

    // Read joystick (example: left stick for translation, right X for rotation)
    double x = -driver1.GetRawAxis(0); // forward
    double y = -driver1.GetRawAxis(1);  // strafe
    double rotation = -driver1.GetRawAxis(4); // rotation input

    swerveDrive->Move(x, y, rotation);
    
    swerveDrive->Update();
    

    // ################################################ EDIT DRIVER HERE ################################################

    BryceDrive();


    intakeModule->Update();
    turretModule->UpdateState(turretModule->Shoot);
    turretModule->Update(odometry->GetPose(), frc::ChassisSpeeds(), x, y, rotation);
    

}

void Robot::TeleopExit() {}

void Robot::TestInit() 
{
    odometry->ResetPose(frc::Pose2d(2_m, 4_m, frc::Rotation2d(0_deg)));
}

void Robot::TestPeriodic() 
{
    frc::Pose2d currentPosition = odometry->GetPose();
    double headingInRadians = currentPosition.Rotation().Radians().value();
    // shooter is 9" back form the center. Use trig to add that 9" 
    frc::Pose2d shooerPosition(currentPosition.X() - (9_in * std::cos(headingInRadians)), currentPosition.Y() - (9_in * std::sin(headingInRadians)), currentPosition.Rotation());
    
    goalDistance = std::sqrt(std::pow(Constants::goalPosition.X().value() - currentPosition.X().value(), 2) + std::pow(Constants::goalPosition.Y().value() - currentPosition.Y().value(), 2)) * 1000 / 25.4;

    // Read joystick (example: left stick for translation, right X for rotation)
    double x = -driver1.GetRawAxis(0); // forward
    double y = -driver1.GetRawAxis(1);  // strafe
    double rotation = -driver1.GetRawAxis(4); // rotation input

    swerveDrive->Move(x, y, rotation);
    
    swerveDrive->Update();



    if (driver1.GetRawButton(1))
    {
        timer->Reset();
        shooterModule->ShootAtDistance(goalDistance);
        going = true;
    }
    if (going)
    {
        timer->Update();
        if (timer->GetStartTime() > 3 && timer->GetStartTime() < 10)
        {
            funnelModule->UpdateState(funnelModule->Feed);
        }
        else if (timer->GetStartTime() > 10)
        {
            funnelModule->UpdateState(funnelModule->Idle);
            shooterModule->Stop();
            going = false;
        }
    }
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
        intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        shooterModule->ShootAtDistance(0);
    }
    else if (driver1.GetRawButton(6)) // Right Bumper ################ Pass
    {
        intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        shooterModule->PassBall();
    }
    else if (driver1.GetRawAxis(2)) // Left Trigger ################ Intake 
    {
        intakeModule->UpdateState(intakeModule->Intaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else if (driver1.GetRawButton(7)) // Left Bumper ################ Outake
    {
        intakeModule->UpdateState(intakeModule->Outaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else
    {
        intakeModule->UpdateState(intakeModule->Idle);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }

    
    if (driver1.GetRawButtonPressed(2)) 
    {
        intakePivotToggle = !intakePivotToggle;
        intakeModule->SetPivot(intakeModule->Up);
        
        intakePivotTogglePressed = true;
    }

    if (intakePivotTogglePressed)
    {
        if (intakePivotToggle)
        {
            intakeModule->SetPivot(intakeModule->Half);
        }
        else
        {
            intakeModule->SetPivot(intakeModule->Down);
        }
    }

    if (driver1.GetRawButtonPressed(4))
    {
        intakePivotTogglePressed = false;
        intakeModule->SetPivot(intakeModule->Up);
    }

}

void Robot::GabeDrive() 
{
    // Left Bumper: Switch Speeds
    // RIght Bottom Pattle: Intake Out


    
    if (driver1.GetRawButtonPressed(8))
    {
        swerveDrive->ResetYaw();
    }


    if (driver1.GetRawAxis(3)) // Right Trigger ################ Shoot 
    {
        intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        shooterModule->ShootAtDistance(0);
    }
    else if (driver1.GetRawButton(6)) // Right Bumper ################ Pass
    {
        intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        shooterModule->PassBall();
    }
    else if (driver1.GetRawAxis(2)) // Left Trigger ################ Intake 
    {
        intakeModule->UpdateState(intakeModule->Intaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else if (driver1.GetRawButton(1)) // Left Pattle ################ Outake
    {
        intakeModule->UpdateState(intakeModule->Outaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else
    {
        intakeModule->UpdateState(intakeModule->Idle);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }

    
    if (driver1.GetRawButtonPressed(-1)) 
    {
        intakeModule->SetPivot(intakeModule->Up);
    }
    else if (driver1.GetRawButtonPressed(-1))
    {
        intakeModule->SetPivot(intakeModule->Half);
    }
    else if (driver1.GetRawButtonPressed(-1))
    {
        intakeModule->SetPivot(intakeModule->Down);
    }

}

void Robot::AsherDrive() 
{
    if (driver1.GetRawButtonPressed(7))
    {
        swerveDrive->ResetYaw();
        odometry->ResetPose(frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(0_deg)));
    }


    
    if (driver1.GetRawAxis(3)) // Right Trigger ################ Shoot 
    {
        intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        shooterModule->ShootAtDistance(goalDistance);

        // TO;DO ADD AUTO TRACKING ################ GABE
    }
    else if (driver1.GetRawButton(6)) // Right Bumper ################ Pass
    {
        intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        shooterModule->PassBall();

        // TO;DO ADD AUTO TRACKING ################ GABE
    }
    else if (driver1.GetRawAxis(2)) // Left Trigger ################ Intake 
    {
        intakeModule->UpdateState(intakeModule->Intaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else if (driver1.GetRawButton(7)) // Left Bumper ################ Outake
    {
        intakeModule->UpdateState(intakeModule->Outaking);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }
    else
    {
        intakeModule->UpdateState(intakeModule->Idle);
        funnelModule->UpdateState(funnelModule->Idle);
        shooterModule->Stop();
    }

    if (driver1.GetRawButtonPressed(4)) 
    {
        intakeModule->SetPivot(intakeModule->Up);
    }
    else if (driver1.GetRawButtonPressed(2))
    {
        intakeModule->SetPivot(intakeModule->Half);
    }
    else if (driver1.GetRawButtonPressed(1))
    {
        intakeModule->SetPivot(intakeModule->Down);
    }

    intakeModule->Update();
    
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
