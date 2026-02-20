#pragma once

#include "Core/PIDConfig.h"
#include "Core/Vector2.h"
#include "math.h"

#include <cmath>

namespace Constants
{
    static constexpr double RADIANS_TO_DEGREES = 180.0 / std::numbers::pi;
    static constexpr double DEGREES_TO_RADIANS = std::numbers::pi / 180.0;
    
    namespace Swerve
    {
        // inline Core::PIDConfig turnPIDConfig(1, 120, 5, 3, 0.05); // Air
        inline Core::PIDConfig turnPIDConfig(1, 1, 0, 0, 0); // Ground
        inline Core::PIDConfig turningWhileMovingPIDConfig(1, 0.9, 0, 0, 0.2); // Ground
        inline Core::PIDConfig moduleTurnPIDConfig(1, 120, 0.01, 3, 0.05); // Ground
        inline constexpr char pigeonID = 14;

        // Meters
        inline constexpr double MODULE_X_OFFSET = 0.62865 / 2;
        inline constexpr double MODULE_Z_OFFSET = 0.62865 / 2;
        inline constexpr double DIAGONAL_MODULE_OFFSET = 0.44452;

        inline constexpr double WHEEL_CIRCUMFRANCE = 95.25 / 1000 * std::numbers::pi; // 2r * pi
        inline constexpr double DRIVE_GEAR_RATIO = 6.75;
        inline constexpr double MOTOR_TICKS_PER_REVOLUTION = 1;

        inline constexpr double METERS_PER_TICK = WHEEL_CIRCUMFRANCE / (MOTOR_TICKS_PER_REVOLUTION * DRIVE_GEAR_RATIO);

        
        namespace FrontRightPod
        {
            inline constexpr char encoderID = 0;
            inline constexpr double encoderOffset = 5.977;
            inline constexpr char driveMotorId = 2;
            inline constexpr char turnMotorId = 1;
            
            inline constexpr double MODULE_X_POSITION = 9.875 * 25.4; // DIstance from the center of the robot to the middle of the swerve modle on the x axis (units are in CM)
            inline constexpr double MODULE_Y_POSITION = 12.375 * 25.4; // DIstance from the center of the robot to the middle of the swerve modle on the y axis (units are in CM)
            static inline double MODULE_DISTANCE_FROM_PIVOT = std::sqrt((MODULE_X_POSITION * MODULE_X_POSITION) + (MODULE_Y_POSITION * MODULE_Y_POSITION)) ;
            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(90 + (std::atan2(MODULE_X_POSITION, MODULE_Y_POSITION) * RADIANS_TO_DEGREES), 1); // 315
        }
        namespace FrontLeftPod
        {
            inline constexpr char encoderID = 1;
            inline constexpr double encoderOffset = 292.324;
            inline constexpr char driveMotorId = 4;
            inline constexpr char turnMotorId = 3;

            inline constexpr double MODULE_X_POSITION = -9.875 * 25.4; // DIstance from the center of the robot to the middle of the swerve modle on the x axis (units are in CM)
            inline constexpr double MODULE_Y_POSITION = 12.375 * 25.4; // DIstance from the center of the robot to the middle of the swerve modle on the y axis (units are in CM)
            static inline double MODULE_DISTANCE_FROM_PIVOT = std::sqrt((MODULE_X_POSITION * MODULE_X_POSITION) + (MODULE_Y_POSITION * MODULE_Y_POSITION)) ;
            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(std::atan2(MODULE_X_POSITION, MODULE_Y_POSITION), 1); // 45
        }
        namespace BackLeftPod
        {
            inline constexpr char encoderID = 2;
            inline constexpr double encoderOffset = 219.8145;
            inline constexpr char driveMotorId = 6;
            inline constexpr char turnMotorId = 5;

            inline constexpr double MODULE_X_POSITION = -9.875 * 25.4; // DIstance from the center of the robot to the middle of the swerve modle on the x axis (units are in CM)
            inline constexpr double MODULE_Y_POSITION = -12.375 * 25.4; // DIstance from the center of the robot to the middle of the swerve modle on the y axis (units are in CM)
            static inline double MODULE_DISTANCE_FROM_PIVOT = std::sqrt((MODULE_X_POSITION * MODULE_X_POSITION) + (MODULE_Y_POSITION * MODULE_Y_POSITION)) ;
            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(std::atan2(MODULE_X_POSITION, MODULE_Y_POSITION), 1); // 135
        }
        namespace BackRightPod
        {
            inline constexpr char encoderID = 3;
            inline constexpr double encoderOffset = 118.213;
            inline constexpr char driveMotorId = 8;
            inline constexpr char turnMotorId = 7;

            inline constexpr double MODULE_X_POSITION = 9.875 * 25.4; // DIstance from the center of the robot to the middle of the swerve modle on the x axis (units are in CM)
            inline constexpr double MODULE_Y_POSITION = -12.375 * 25.4; // DIstance from the center of the robot to the middle of the swerve modle on the y axis (units are in CM)
            static inline double MODULE_DISTANCE_FROM_PIVOT = std::sqrt((MODULE_X_POSITION * MODULE_X_POSITION) + (MODULE_Y_POSITION * MODULE_Y_POSITION)) ;
            static inline Core::Vector2 TURN_VECTOR = Core::Vector2::CreateAngularVector(std::atan2(MODULE_X_POSITION, MODULE_Y_POSITION), 1); // 225
        }
    }

    static constexpr char* CANIVOUR_NAME = "CANivour";
}