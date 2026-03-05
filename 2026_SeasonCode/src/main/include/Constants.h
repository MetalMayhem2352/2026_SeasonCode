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
            .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast)
        )
        .WithCurrentLimits(
            ctre::phoenix6::configs::CurrentLimitsConfigs{}
            .WithStatorCurrentLimit(40_A)
            .WithStatorCurrentLimitEnable(true)
        );
    
    namespace Swerve
    {
        // inline Core::PIDConfig turnPIDConfig(1, 120, 5, 3, 0.05); // Air
        inline Core::PIDConfig turnPIDConfig(1, 1, 0, 0, 0); // Ground
        inline Core::PIDConfig turningWhileMovingPIDConfig(1, 0.9, 0, 0, 0.2); // Ground
        inline Core::PIDConfig moduleTurnPIDConfig(1, 120, 0.01, 3, 0.05); // Ground
        inline constexpr char pigeonID = 0;

        // Meters
        inline constexpr double MODULE_X_POSITION = 9.875 * 2.54 / 100; // Distance from the center of the robot to the middle of the swerve modle on the x axis
        inline constexpr double MODULE_Y_POSITION = 12.375 * 2.54 / 100; // Distance from the center of the robot to the middle of the swerve modle on the y axis
        static inline double DIAGONAL_MODULE_OFFSET = std::sqrt((MODULE_X_POSITION * MODULE_X_POSITION) + (MODULE_Y_POSITION * MODULE_Y_POSITION)) ;
            
        inline constexpr double WHEEL_CIRCUMFRANCE = 2 * (2 * 2.54 / 100) * std::numbers::pi; // 2r * pi
        inline constexpr double DRIVE_GEAR_RATIO = 63.0 / 109.0;
        inline constexpr double MOTOR_TICKS_PER_REVOLUTION = 1;

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

        namespace FrontLeftPod
        {
            inline constexpr char encoderID = 20;
            inline constexpr double encoderOffset = 292.324;
            inline constexpr char driveMotorId = 1;
            inline constexpr char turnMotorId = 0;

            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(std::atan2(-MODULE_X_POSITION, MODULE_Y_POSITION), 1); // 45
        }
        namespace FrontRightPod
        {
            inline constexpr char encoderID = 21;
            inline constexpr double encoderOffset = 5.977;
            inline constexpr char driveMotorId = 2;
            inline constexpr char turnMotorId = 1;
            
            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(90 + (std::atan2(MODULE_X_POSITION, MODULE_Y_POSITION) * RADIANS_TO_DEGREES), 1); // 315
        }
        namespace BackRightPod
        {
            inline constexpr char encoderID = 22;
            inline constexpr double encoderOffset = 118.213;
            inline constexpr char driveMotorId = 5;
            inline constexpr char turnMotorId = 4;

            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(std::atan2(MODULE_X_POSITION, -MODULE_Y_POSITION), 1); // 225
        }
        namespace BackLeftPod
        {
            inline constexpr char encoderID = 23;
            inline constexpr double encoderOffset = 219.8145;
            inline constexpr char driveMotorId = 7;
            inline constexpr char turnMotorId = 6;

            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(std::atan2(-MODULE_X_POSITION, -MODULE_Y_POSITION), 1); // 135
        }
    }

    namespace Turret
    {
        inline constexpr int turretID = 14; 
        static inline Core::PIDConfig TurretPIDConfig(0.12, 25, 0.01, 15, 0.00001);
        
        
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration turretMotor =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );
        
    }

    namespace Shooter
    {
        inline constexpr int shooterID = 15; 
        inline constexpr int hoodID = 16; 


        static constexpr ctre::phoenix6::configs::TalonFXConfiguration shooterMotorCondiguration =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration hoodMotorCongiuration =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );
    }

    namespace Intake
    {
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration topIntakeMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration basketIntakeMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration groundIntakeMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration intakePivotMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );

        inline constexpr int TOP_INTAKE_ID = 8; 
        inline constexpr int BASKET_INTAKE_ID = 9; 
        inline constexpr int GROUND_INTAKE_ID = 10; 
        inline constexpr int PIVOT_ID = 11; 
    
        
        static inline Core::PIDConfig PivotPIDConfig(0.175, 0.2, 0.5, 0.2, 0);
        
        inline constexpr double GROUND_PIVOT_POSITION = 0.0;
        inline constexpr double SHOOT_PIVOT_POSITION = -0.22; 
    }

    namespace Basket
    {
        inline Core::PIDConfig basketPIDConfig(0.5, 360, 0, 0, 0);
        
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration leftBasketMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration rightBasketMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );

        inline constexpr int LEFT_BASKET_ID = 12; 
        inline constexpr int RIGHT_BASKET_ID = 13; 

        inline constexpr int UP_POSITION = 0; 
        inline constexpr int DOWN_POSITON = -1800; 
    }

    static std::string CANIVOUR_NAME = "Default Name";
}