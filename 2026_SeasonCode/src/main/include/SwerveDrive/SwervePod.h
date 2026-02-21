#pragma once

#include "SwerveEncoder.h"
#include "Core/PIDController.h"
#include "Core/Vector2.h"
#include "Core/Timer.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>

namespace SwerveDrive
{
    class SwervePod
    {
    private:
        Core::PIDController* turnPIDController;
        Core::Timer* timer;

        ctre::phoenix6::hardware::TalonFX* turnMotor2;
        rev::spark::SparkMax* turnMotor;
        SwerveEncoder* encoder;
        ctre::phoenix6::hardware::TalonFX* driveMotor2;
        rev::spark::SparkMax* driveMotor;

        double targetAngle = 0;

        double angleDelta = 0;
        double currentAngle = 0;

        double lastDrivePosition;

        void Turn(double angle);

    public:
        rev::spark::SparkRelativeEncoder* driveEncoder;

        SwervePod(char encoderId, double encoderOffset, char driveMotorId, char turnMotorId, Core::PIDConfig turnPIDConfig);
        ~SwervePod();
        
        
        void RverseMotors(bool driveMotor, bool turnMotor);
        
        double GetAngle();
        double GetAngleDelta();
        double GetMovementDelta();

        void Move(double angle, double power);
        void Update();
    };
}