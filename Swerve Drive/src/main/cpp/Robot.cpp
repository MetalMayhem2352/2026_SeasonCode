// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandScheduler.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>

Robot::Robot() 
{
    networkTableModule = new Modules::NetworkTableModule();

    swerveDrive = new Pathing::CTRESwerveDrive();

    odometry = swerveDrive->GetOdometery();

    funnelModule = new Modules::FunnelModule();
    intakeModule = new Modules::IntakeModule();
    shooterModule = new Modules::ShooterModule();
    // turretModule = new Modules::NewTurretModule();
    basketModule = new Modules::BasketModule();


    timer = new Core::Timer();
    autoTimer = new Core::Timer();

    
    m_chooser.SetDefaultOption(kAutoNameBump, kAutoNameBump);
    m_chooser.AddOption(kAutoNameTrench, kAutoNameTrench);
    m_chooser.AddOption(kAutoNameHub, kAutoNameHub);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);



    swerveDrive->ResetYaw();
}


Robot::~Robot() 
{
    delete(swerveDrive);
    
    delete(funnelModule);
    delete(intakeModule);
    delete(shooterModule);
    // delete(turretModule);
    delete(basketModule);

    delete(networkTableModule);

    delete(timer);
    delete(autoTimer);
}

int i = 0;
void Robot::RobotPeriodic() 
{
    i++;

    if (i % 50 == 0)
    {
    }

    networkTableModule->Update();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{
    
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    

    autoTimer->Reset();
}

void Robot::AutonomousPeriodic() 
{
    BumpAuto(); 
    m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString("Auto Selector",
    //     kAutoNameDefault);
    wpi::print("Auto selected: {}\n", m_autoSelected);

    if (m_autoSelected == kAutoNameTrench) 
    {
        TrenchAuto();
    }
    else if (m_autoSelected == kAutoNameBump) 
    {
        BumpAuto();
    }
    else if (m_autoSelected == kAutoNameHub) 
    {
        HubAuto();
    }
    else
    {
        BumpAuto();
    }
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    intakePivotTogglePressed = false;
    intakePivotToggle = false;
    
}

void Robot::TeleopPeriodic() 
{

    // Read joystick (example: left stick for translation, right X for rotation)
    x = -driver1.GetRawAxis(0); // forward
    y = -driver1.GetRawAxis(1);  // strafe
    rotation = -driver1.GetRawAxis(4); // rotation input

    swerveDrive->Move(x, y, rotation);
    
    swerveDrive->Update();
    

    // ################################################ EDIT DRIVER HERE ################################################

    BryceDrive();
    

    SeccondDriveAim();


    intakeModule->Update();
    // turretModule->Update(odometry->GetPose(), frc::ChassisSpeeds(), x, y, rotation);

}

void Robot::TeleopExit() 
{
    isPreparingShooting = false;
}

void Robot::TestInit() 
{
    odometry->ResetPose(frc::Pose2d(2_m, 4_m, frc::Rotation2d(0_deg)));
}

void Robot::Test2()
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
        isPreparingShooting = true;
    }
    if (isPreparingShooting)
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
            isPreparingShooting = false;
        }
    }
}

void Robot::TestPeriodic() 
{
    intakeModule->UpdateState(intakeModule->Intaking);
}

void Robot::TestExit() 
{

}


void Robot::BryceDrive() 
{
    if (driver1.GetRawButtonPressed(3))
    {
        isPreparingShooting = true;
    }


    // RIght Bottom Pattle: Intake Out
    if (driver1.GetPOV() == 90)
    {
        swerveDrive->ResetYaw();
        odometry->ResetPose(frc::Pose2d(18_in, 15.5_in, frc::Rotation2d(0_deg)));
    }
    

    if (driver1.GetRawAxis(3)) // Right Trigger ################ Shoot 
    {
        intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);
        
        // turretModule->UpdateState(turretModule->Shoot);
    }
    else if (driver1.GetRawButton(6)) // Right Bumper ################ Pass
    {
        intakeModule->UpdateState(intakeModule->Shooting);
        funnelModule->UpdateState(funnelModule->Feed);

        // turretModule->UpdateState(turretModule->PassRight);
    }
    else if (driver1.GetRawAxis(2)) // Left Trigger ################ Intake 
    {
        intakeModule->UpdateState(intakeModule->Intaking);
        funnelModule->UpdateState(funnelModule->Idle);
    }
    else if (driver1.GetRawButton(5)) // Left Bumper ################ Outake
    {
        intakeModule->UpdateState(intakeModule->Outaking);
        funnelModule->UpdateState(funnelModule->Unjam);
    }
    else
    {
        intakeModule->UpdateState(intakeModule->Idle);
        funnelModule->UpdateState(funnelModule->Idle);
    }

    if (driver1.GetRawButtonPressed(1))
    {
        armUp = !armUp;
        if (armUp)
        {
            basketModule->UpdateState(basketModule->Up);
        }
        else
        {
            basketModule->UpdateState(basketModule->Down);
        }
    }
    
    if (driver1.GetRawButtonPressed(2)) 
    {
        intakePivotToggle = !intakePivotToggle;
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

    if (isPreparingShooting)
    {
        shooterModule->ShootAtDistance(shooterPower);
    }
    else
    {
        shooterModule->Stop();
    }

    if (!driver1.GetRawAxis(3) && isShooting)
    {
        isShooting = false;
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

void Robot::SeccondDriveAim() 
{   
    double leftY = -driver2.GetRawAxis(1);

    double rightX = driver2.GetRawAxis(4);
    double rightY = -driver2.GetRawAxis(5);

    if (driver2.GetRawButton(1))
    {
        isPreparingShooting = true;
    }
    else if (driver2.GetPOV() != -1)
    {
        std::cout << "LAG!\n";
        isPreparingShooting = false;
    }

    shooterModule->MoveHood(10 + (leftY * 10));

    if (driver2.GetRawAxis(3) > 0.25)
    {
        shooterPower = (driver2.GetRawAxis(3) / 0.75) * 0.4 + 0.6;
    }
    else if (driver2.GetRawAxis(2))
    {
        shooterPower = 0.6;
    }
    else if (driver2.GetRawButton(3))
    {
        shooterPower = 0.7;
    }
    else if (driver2.GetRawButton(5))
    {
        shooterPower = 0.8;
    }
    else if (driver2.GetRawButton(6))
    {
        shooterPower = 0.9;
    }
    else if (driver2.GetRawButton(4))
    {
        shooterPower = 1;
    }
    
}


void Robot::BumpAuto()
{
    intakeModule->SetPivot(intakeModule->Down);

    autoTimer->Update();

    shooterModule->ShootAtDistance(0.8);

    if (autoTimer->GetStartTime() > 3)
    {
        funnelModule->UpdateState(funnelModule->Feed);
        intakeModule->UpdateState(intakeModule->IntakeAndShoot);
    }

    intakeModule->Update();
}

void Robot::TrenchAuto()
{
    intakeModule->SetPivot(intakeModule->Down);

    autoTimer->Update();

    shooterModule->MoveHood(10);
    shooterModule->ShootAtDistance(1);

    if (autoTimer->GetStartTime() > 3)
    {
        funnelModule->UpdateState(funnelModule->Feed);
        intakeModule->UpdateState(intakeModule->IntakeAndShoot);
    }

    intakeModule->Update();
}

void Robot::HubAuto()
{
    autoTimer->Update();

    if (autoTimer->GetStartTime() < 0.5)
    {
        swerveDrive->Move(0, -0.25, 0);
    }


    

    intakeModule->SetPivot(intakeModule->Down);


    shooterModule->MoveHood(0);
    shooterModule->ShootAtDistance(0.8);

    if (autoTimer->GetStartTime() > 3)
    {
        funnelModule->UpdateState(funnelModule->Feed);
        intakeModule->UpdateState(intakeModule->IntakeAndShoot);
    }

    intakeModule->Update();
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
