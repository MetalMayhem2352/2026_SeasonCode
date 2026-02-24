#pragma once

// Custom
#include "Core/PIDConfig.h"
#include "Core/Vector2.h"

// FRC
#include <ctre/phoenix6/core/CoreTalonFX.hpp>

// C++
#include "math.h"
#include <cmath>

namespace Constants
{
    static constexpr double RADIANS_TO_DEGREES = 180.0 / std::numbers::pi;
    static constexpr double DEGREES_TO_RADIANS = std::numbers::pi / 180.0;

    static constexpr ctre::phoenix6::configs::TalonFXConfiguration commonConfigs =
        ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithMotorOutput(
            ctre::phoenix6::configs::MotorOutputConfigs{}
            .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        )
        .WithCurrentLimits(
            ctre::phoenix6::configs::CurrentLimitsConfigs{}
            .WithStatorCurrentLimit(120_A)
            .WithStatorCurrentLimitEnable(true)
        );
    
    namespace Swerve
    {
        // inline Core::PIDConfig turnPIDConfig(1, 120, 5, 3, 0.05); // Air
        inline Core::PIDConfig turnPIDConfig(1, 1, 0, 0, 0); // Ground
        inline Core::PIDConfig turningWhileMovingPIDConfig(1, 0.9, 0, 0, 0.2); // Ground
        inline Core::PIDConfig moduleTurnPIDConfig(1, 120, 0.01, 3, 0.05); // Ground
        inline constexpr char pigeonID = 14;

        // Meters
        inline constexpr double MODULE_X_POSITION = 9.875 * 2.54 / 100; // Distance from the center of the robot to the middle of the swerve modle on the x axis
        inline constexpr double MODULE_Y_POSITION = 12.375 * 2.54 / 100; // Distance from the center of the robot to the middle of the swerve modle on the y axis
        static inline double DIAGONAL_MODULE_OFFSET = std::sqrt((MODULE_X_POSITION * MODULE_X_POSITION) + (MODULE_Y_POSITION * MODULE_Y_POSITION)) ;
            
        inline constexpr double WHEEL_CIRCUMFRANCE = 2 * (4 * 2.54 / 100) * std::numbers::pi; // 2r * pi
        inline constexpr double DRIVE_GEAR_RATIO = 63.0 / 109.0;
        inline constexpr double MOTOR_TICKS_PER_REVOLUTION = 360;

        inline constexpr double METERS_PER_TICK = WHEEL_CIRCUMFRANCE / (MOTOR_TICKS_PER_REVOLUTION * DRIVE_GEAR_RATIO);
        
        


        static constexpr ctre::phoenix6::configs::TalonFXConfiguration turnMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration driveMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );

        namespace FrontRightPod
        {
            inline constexpr char encoderID = 0;
            inline constexpr double encoderOffset = 5.977;
            inline constexpr char driveMotorId = 2;
            inline constexpr char turnMotorId = 1;
            
            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(90 + (std::atan2(MODULE_X_POSITION, MODULE_Y_POSITION) * RADIANS_TO_DEGREES), 1); // 315
        }
        namespace FrontLeftPod
        {
            inline constexpr char encoderID = 1;
            inline constexpr double encoderOffset = 292.324;
            inline constexpr char driveMotorId = 4;
            inline constexpr char turnMotorId = 3;

            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(std::atan2(-MODULE_X_POSITION, MODULE_Y_POSITION), 1); // 45
        }
        namespace BackLeftPod
        {
            inline constexpr char encoderID = 2;
            inline constexpr double encoderOffset = 219.8145;
            inline constexpr char driveMotorId = 6;
            inline constexpr char turnMotorId = 5;

            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(std::atan2(-MODULE_X_POSITION, -MODULE_Y_POSITION), 1); // 135
        }
        namespace BackRightPod
        {
            inline constexpr char encoderID = 3;
            inline constexpr double encoderOffset = 118.213;
            inline constexpr char driveMotorId = 8;
            inline constexpr char turnMotorId = 7;

            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(std::atan2(MODULE_X_POSITION, -MODULE_Y_POSITION), 1); // 225
        }
    }

    namespace Turret
    {
        inline constexpr int turretID = 1; 
        static inline Core::PIDConfig TurretPIDConfig(1,360,0.2,5,0.4);

        static constexpr ctre::phoenix6::configs::TalonFXConfiguration shooterMotor2 =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );
    }


    static constexpr char* CANIVOUR_NAME = "CANivour";
}