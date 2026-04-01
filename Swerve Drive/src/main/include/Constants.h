#pragma once

// Custom
#include "Core/PIDConfig.h"
#include "Core/Vector2.h"
#include "Core/PiecewiseLinearFunction.h"

// FRC
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/geometry/Pose2d.h>

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

    static frc::Pose2d goalPosition(4.75_m, 4.25_m, frc::Rotation2d());
    static frc::Pose2d zeroPosition(0_m, -2_m, frc::Rotation2d());

    namespace Turret
    {
        inline constexpr int turretID = 14; 
        //static inline Core::PIDConfig TurretPIDConfig(0.6, 2, 0, 0, 0);
        static inline Core::PIDConfig TurretPIDConfig2(0.25, 20, 0, 0, 0);
        
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
        inline constexpr int SHOOTER_ID = 15; 
        inline constexpr int HOOD_ID = 9;

        inline constexpr float HOOD_MIN_DOWN_POS = 0;
        inline constexpr float HOOD_MAX_UP_POS = 0.72;
        
        inline constexpr float HOOD_MIN_DOWN_ANGLE = 0;
        inline constexpr float HOOD_MAX_UP_ANGLE = 20;

        // Hood Degrees: Servo Position
        static Core::PiecewiseLinearFunction HOOD_TABLE(
        {
            {HOOD_MIN_DOWN_ANGLE, HOOD_MIN_DOWN_POS},
            {HOOD_MAX_UP_ANGLE, HOOD_MAX_UP_POS}
        });


        static constexpr ctre::phoenix6::configs::TalonFXConfiguration shooterMotorCondiguration =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );
        
        static std::string SHOOTING_DISTANCE_LOOKUP_TABLE_NAME = "ShooingDistanceLookupTable.json";
    }

    namespace Intake
    {
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration leftPivotMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration rightPivotMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );
        
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration frontIntakeMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );
        static constexpr ctre::phoenix6::configs::TalonFXConfiguration backIntakeMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );

        static constexpr ctre::phoenix6::configs::TalonFXConfiguration basketIntakeMotorConfig =
            ctre::phoenix6::configs::TalonFXConfiguration{commonConfigs}
            .WithMotorOutput(
                ctre::phoenix6::configs::MotorOutputConfigs{commonConfigs.MotorOutput}
                .WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive)
        );

        inline constexpr int LEFT_PIVOT_ID = 12; 
        inline constexpr int RIGHT_PIVOT_ID = 11; 
        inline constexpr int PIVOT_ENCODER_ID = 0; 

        inline constexpr int FRONT_INTAKE_ID = 8; 
        inline constexpr int BACK_INTAKE_ID = 9; 
        inline constexpr int BASKET_INTAKE_ID = 10; 


        inline constexpr float UP_ENCODER_POSITION = 0.590; 
        inline constexpr float DOWN_ENCODER_POSITION = 0.180; 
        inline constexpr float HALF_ENCODER_POSITION = 0.38; 

        
        static inline Core::PIDConfig PIVOT_PID_CONFIG(0.5, 0.5, 0, 0, 0);
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


    inline constexpr char pigeonID = 24;
    static std::string CANIVOUR_NAME = "Default Name";
    static std::string HOME_DIRECTORY = "/home/lvuser/";
}