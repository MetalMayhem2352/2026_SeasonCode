#pragma once

#include "SwerveEncoder.h"
#include "Core/PIDController.h"
#include "Core/Vector2.h"
#include "Core/Timer.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>

namespace CustomSwerveDrive
{
    class SwervePod
    {
    private:
        Core::PIDController* turnPIDController;
        Core::Timer* timer;

        ctre::phoenix6::hardware::TalonFX* turnMotor;
        SwerveEncoder* encoder;
        ctre::phoenix6::hardware::TalonFX* driveMotor;

        double targetAngle = 0;

        double angleDelta = 0;
        double currentAngle = 0;

        double lastDrivePosition;
        double drivePosition;

        void Turn(double angle);

    public:

        SwervePod(char encoderId, double encoderOffset, char driveMotorId, char turnMotorId, Core::PIDConfig turnPIDConfig);
        ~SwervePod();
        
        
        double GetAngle();
        double GetAngleDelta();
        double GetMovementDelta();

        void Move(double angle, double power);
        void Update();
    };
}