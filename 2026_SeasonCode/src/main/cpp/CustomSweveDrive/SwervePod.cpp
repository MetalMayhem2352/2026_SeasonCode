#include "CustomSwerveDrive/SwervePod.h"
#include "Constants.h"

#include <iostream>

namespace CustomSwerveDrive
{
    SwervePod::SwervePod(int encoderId, double encoderOffset, char driveMotorId, ctre::phoenix6::configs::TalonFXConfiguration driveMotorConfig, char turnMotorId, Core::PIDConfig turnPIDConfig)
    {
        encoder = new SwerveEncoder(encoderId);
        driveMotor = new ctre::phoenix6::hardware::TalonFX(driveMotorId, Constants::CANIVOUR_NAME);
        turnMotor = new ctre::phoenix6::hardware::TalonFX(turnMotorId, Constants::CANIVOUR_NAME);

        encoder->SetOffset(encoderOffset);
        encoder->SetInverted(true);
        
        turnMotor->GetConfigurator().Apply(Constants::Swerve::turnMotorConfig);
        driveMotor->GetConfigurator().Apply(driveMotorConfig);

        turnPIDController = new Core::PIDController(turnPIDConfig);
        turnPIDController->SetLoop(true, 0, 360);

        timer = new Core::Timer();
    }

    SwervePod::~SwervePod()
    {
        delete(encoder);
        delete(driveMotor);
        delete(turnMotor);

        delete(turnPIDController);

        delete(timer);
    }

    double SwervePod::GetAngle()
    {
        return encoder->GetAngle();
    }
    double SwervePod::GetAngleDelta()
    {
        return angleDelta;
    }
    double SwervePod::GetMovementDelta()
    {
        return drivePosition - lastDrivePosition;
    }


    void SwervePod::Turn(double targetAngle)
    {
        turnMotor->Set(turnPIDController->Calculate(encoder->GetAngle(), targetAngle, timer->GetDeltaTime()));
    }
    void SwervePod::Move(double angle, double power)
    {
        /*
        std::cout << "currentAngle: " << GetAngle() << "\n";
        */
        if (power < 0.1 && power > -0.1)
        {
            Turn(targetAngle);
            driveMotor->Set(0);
            return;
        }

        
        bool dirrect = true;
        double currentAngle = GetAngle();
        
        if (currentAngle > 180)
        {
            currentAngle -= 360;
        }
        if (angle > 180)
        { 
            angle -= 360;
        }

        double reverseTarget = angle < 0 ? angle + 180 : angle - 180;

        double dirrectAngle = std::abs(angle < 0 ? angle - currentAngle : angle + currentAngle);
        double reverseAngle = std::abs(reverseTarget < 0 ? reverseTarget - currentAngle : reverseTarget + currentAngle);
        
        std::cout << "currentAngle: " << currentAngle << '\n';
        std::cout << "angle: " << angle << '\n';
        std::cout << "reverseTarget: " << reverseTarget << "\n\n";
        std::cout << "dirrectAngle: " << dirrectAngle << '\n';
        std::cout << "reverseAngle: " << reverseAngle << '\n';
  
        //if (reverseAngle < dirrectAngle)
        //{
        //    dirrect = false;
        //}
        //else
        //{
            dirrect = true;
        //}

        if (dirrect)
        {
            targetAngle = angle;
            // driveMotor->Set(power);
        
        }
        else
        {
            targetAngle = reverseTarget;
            // driveMotor->Set(-power);
        }
        
        // Turn(targetAngle);
    }
    void SwervePod::Update()
    {
        lastDrivePosition = drivePosition;
        drivePosition = driveMotor->GetPosition().GetValue().value();
        
        angleDelta = currentAngle; // Temporally changing value to be last angle
        currentAngle = encoder->GetAngle();
        angleDelta -= currentAngle; // Switching back to angle delta

        timer->Update();
    }
}