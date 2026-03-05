#include "CustomSwerveDrive/SwerveDriveModule.h"
#include "Core/Vector2.h"
#include <iostream>

namespace CustomSwerveDrive
{
    SwerveDriveModule::SwerveDriveModule()
    {
        frontRightPod = new SwervePod(Constants::Swerve::FrontRightPod::encoderID, Constants::Swerve::FrontRightPod::encoderOffset, Constants::Swerve::FrontRightPod::driveMotorId, Constants::Swerve::FrontRightPod::turnMotorId, Constants::Swerve::moduleTurnPIDConfig);
        frontLeftPod = new SwervePod(Constants::Swerve::FrontLeftPod::encoderID, Constants::Swerve::FrontLeftPod::encoderOffset, Constants::Swerve::FrontLeftPod::driveMotorId, Constants::Swerve::FrontLeftPod::turnMotorId, Constants::Swerve::moduleTurnPIDConfig);
        backLeftPod = new SwervePod(Constants::Swerve::BackLeftPod::encoderID, Constants::Swerve::BackLeftPod::encoderOffset, Constants::Swerve::BackLeftPod::driveMotorId, Constants::Swerve::BackLeftPod::turnMotorId, Constants::Swerve::moduleTurnPIDConfig);
        backRightPod = new SwervePod(Constants::Swerve::BackRightPod::encoderID, Constants::Swerve::BackRightPod::encoderOffset, Constants::Swerve::BackRightPod::driveMotorId, Constants::Swerve::BackRightPod::turnMotorId, Constants::Swerve::moduleTurnPIDConfig);

        pigeon = new ctre::phoenix6::hardware::Pigeon2(Constants::Swerve::pigeonID, Constants::CANIVOUR_NAME);

        turningPIDController = new Core::PIDController(Constants::Swerve::turnPIDConfig);
        turningPIDController->SetLoop(true, -std::numbers::pi, std::numbers::pi);
        
        turningWhileMovingPIDController = new Core::PIDController(Constants::Swerve::turningWhileMovingPIDConfig);
        turningWhileMovingPIDController->SetLoop(true, -std::numbers::pi, std::numbers::pi);

        turningPIDTimer = new Core::Timer();
    }

    SwerveDriveModule::~SwerveDriveModule()
    {
        delete(frontRightPod);
        delete(frontLeftPod);
        delete(backLeftPod);
        delete(backRightPod);

        delete(pigeon);

        delete(turningPIDController);
        delete(turningPIDTimer);
    }

    void SwerveDriveModule::ResetIMU()
    {
        pigeon->Reset();
    }

    void SwerveDriveModule::Move(double x, double z, double yRotation)
    {
//         Core::Vector2 frontRightPower = (Constants::Swerve::FrontRightPod::TURN_VECTOR * yRotation) + Core::Vector2(x, z);
//         Core::Vector2 frontLeftPower = (Constants::Swerve::FrontLeftPod::TURN_VECTOR * yRotation) + Core::Vector2(x, z);
//         Core::Vector2 backLeftPower = (Constants::Swerve::BackLeftPod::TURN_VECTOR * yRotation) + Core::Vector2(x, z);
//         Core::Vector2 backRightPower = (Constants::Swerve::BackRightPod::TURN_VECTOR * yRotation) + Core::Vector2(x, z);

//         double denominator = std::max({frontRightPower.GetMagnitude(), frontLeftPower.GetMagnitude(), backLeftPower.GetMagnitude(), backRightPower.GetMagnitude(), 1.0});

        // frontRightPod->Move(0, 0);
        frontRightPod->Move(0,0/*frontRightPower.GetAngle(), frontRightPower.GetMagnitude() / denominator*/);
//        frontLeftPod->Move(frontLeftPower.GetAngle(), frontLeftPower.GetMagnitude() / denominator);
//        backLeftPod->Move(backLeftPower.GetAngle(), backLeftPower.GetMagnitude() / denominator);
//        backRightPod->Move(backRightPower.GetAngle(), backRightPower.GetMagnitude() / denominator);
    }

    void SwerveDriveModule::MoveRobotCentric(double x, double z, double yRotation)
    {
        Move(x, z, yRotation);
    }

    void SwerveDriveModule::MoveFieldCentric1(double x, double z, double yRotation)
    {
        double heading = GetYaw() * Constants::DEGREES_TO_RADIANS;
        if (heading > std::numbers::pi)
        {
            heading -= std::numbers::pi * 2;
        }

        double relitiveX = (x * std::cos(heading)) - (z * std::sin(heading));
        double relitiveZ = (x * std::sin(heading)) + (z * std::cos(heading));

        Move(relitiveX, relitiveZ, yRotation);
    }
    
    void SwerveDriveModule::MoveFieldCentric2(double xMovement, double zMovement, double headingDegrees)
    {
        double heading = GetYaw() * Constants::DEGREES_TO_RADIANS;
        if (heading > std::numbers::pi)
        {
            heading -= std::numbers::pi * 2;
        }

        double targetHeadingRadians = headingDegrees * Constants::DEGREES_TO_RADIANS;

        double relitiveX = (xMovement * std::cos(heading)) - (zMovement * std::sin(heading));
        double relitiveZ = (xMovement * std::sin(heading)) + (zMovement * std::cos(heading));
        double rotationPower = 0;

        if (std::abs(relitiveX) + std::abs(relitiveZ) > 0.5)
        {
            rotationPower = turningWhileMovingPIDController->Calculate(-heading, targetHeadingRadians, turningPIDTimer->GetDeltaTime()) / 2;
        }
        else
        {
            rotationPower = turningPIDController->Calculate(-heading, targetHeadingRadians, turningPIDTimer->GetDeltaTime()) / 2;
        }

        Move(relitiveX, relitiveZ, rotationPower);
    }

    void SwerveDriveModule::Update()
    {
        frontRightPod->Update();
        frontLeftPod->Update();
        backLeftPod->Update();
        backRightPod->Update();

        turningPIDTimer->Update();
    }

    /// @brief Degrees
    /// @return The YAW of the giro in degrees.
    double SwerveDriveModule::GetYaw()
    {
        double yaw = pigeon->GetYaw().GetValue().value();

        while (yaw < 0)
        {
            yaw += 360;
        }
        return std::fmod(yaw, 360.0);
    }

    
    SwerveDriveOdometry* SwerveDriveModule::CreateSwerveDriveOdometery()
    {
        return new SwerveDriveOdometry(frontRightPod, frontLeftPod, backRightPod, backLeftPod, pigeon);
    }

}