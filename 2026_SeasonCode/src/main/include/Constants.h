#pragma once

// Custom
#include "Core/PIDConfig.h"
#include "Core/Vector2.h"
#include "CustomSwerveDrive/RobotPosition.h"
#include "Auto/Point.h"

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
        inline Core::PIDConfig turningWhileMovingPIDConfig(1, 120, 0, 0, 0); // Ground
        inline Core::PIDConfig moduleTurnPIDConfig(1, 120, 0, 0, 0.0); // Ground
        inline constexpr char pigeonID = 24;

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
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration leftDriveMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
                .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration rightDriveMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
                .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        );

        namespace FrontLeftPod
        {
            inline constexpr int encoderID = 22;
            inline constexpr double encoderOffset = 359.736;
            inline constexpr char driveMotorId = 1;
            inline constexpr char turnMotorId = 0;

            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(38.5892, 1); // 45
        }
        namespace FrontRightPod
        {
            inline constexpr int encoderID = 21;
            inline constexpr double encoderOffset = 357.891;
            inline constexpr char driveMotorId = 3;
            inline constexpr char turnMotorId = 2;
            
            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(141.411, 1); // 315
        }
        namespace BackRightPod
        {
            inline constexpr int encoderID = 22;
            inline constexpr double encoderOffset = 359.824;
            inline constexpr char driveMotorId = 5;
            inline constexpr char turnMotorId = 4;

            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(218.589, 1); // 225
        }
        namespace BackLeftPod
        {
            inline constexpr int encoderID = 23;
            inline constexpr double encoderOffset = 0;
            inline constexpr char driveMotorId = 7;
            inline constexpr char turnMotorId = 6;

            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(321.411, 1); // 135
        }
    }

    namespace Turret
    {
        inline constexpr int turretID = 14; 
        static inline Core::PIDConfig TurretPIDConfig(0.4, 4, 0, 0, 0);
        
        inline constexpr double MAX_ROTATION = 90;
        inline constexpr double MIN_ROTATION = -90;
        inline constexpr double TOLERANCE = 5;
        
        inline constexpr double MAX_MOTOR_ENCODER_POSITION = 4.65;
        inline constexpr double MIN_MOTOR_ENCODER_POSITION = -5.25;

        inline constexpr double MOTOR_TICKS_PER_DEGREE = (MAX_ROTATION - MIN_ROTATION) / (MAX_MOTOR_ENCODER_POSITION - MIN_MOTOR_ENCODER_POSITION);

        
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

        static constexpr ctre::phoenix6::configs::TalonFXConfiguration shooterMotorCondiguration =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );
    }

    namespace Intake
    {
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration frontIntakeMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration backIntakeMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );

        static constexpr ctre::phoenix6::configs::TalonFXConfiguration basketIntakeMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );

        inline constexpr int FRONT_INTAKE_ID = 8; 
        inline constexpr int BACK_INTAKE_ID = 9; 

        inline constexpr int BASKET_INTAKE_ID = 10; 
    }

    namespace Funnel 
    {
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration funnelMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );

        inline constexpr int FUNNEL_MOTOR_ID = 13;
    }


    namespace Auto
    {
        

        static constexpr CustomDriveBase::RobotPosition START_POSE__LEFT_TRENCH{0.65, 4, 0};
        static constexpr CustomDriveBase::RobotPosition START_POSE__LEFT_BUMP{0.65, 4, 0};
        static constexpr CustomDriveBase::RobotPosition START_POSE__RIGHT_BUMP{0.65, 4, 0};
        static constexpr CustomDriveBase::RobotPosition START_POSE__RIGHT_TRENCH{0.65, 4, 0};


        static Autonomous::Point FAR1_LEFT_INTAKE_POSITION{0.5, 7.5, 90};
        static Autonomous::Point FAR1_RIGHT_INTAKE_POSITION{7.5, 7.5, 90};
        
        static Autonomous::Point FAR2_LEFT_INTAKE_POSITION{0.5, 7, 180};
        static Autonomous::Point FAR2_RIGHT_INTAKE_POSITION{7.5, 7, 180};

        static Autonomous::Point FAR3_LEFT_INTAKE_POSITION{0.5, 6.5, 90};
        static Autonomous::Point FAR3_RIGHT_INTAKE_POSITION{7.5, 6.5, 90};

        static Autonomous::Point FAR4_LEFT_INTAKE_POSITION{0.5, 6, 180};
        static Autonomous::Point FAR4_RIGHT_INTAKE_POSITION{7.5, 6, 180};

        
        static Autonomous::Point THROUGH_LEFT_TRENCH_POSITION1{0.3, 3, 0};
        static Autonomous::Point THROUGH_LEFT_TRENCH_POSITION2{0.3, 6, 0};

        
        static Autonomous::Point THROUGH_RIGHT_TRENCH_POSITION1{7.25, 3, 0};
        static Autonomous::Point THROUGH_RIGHT_TRENCH_POSITION2{7.25, 6, 0};
    }


    static std::string CANIVOUR_NAME = "Default Name";
}