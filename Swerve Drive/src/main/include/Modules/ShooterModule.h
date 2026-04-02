#pragma once


#include "Core/PiecewiseLinearFunctionXYZ.h"
#include "Constants.h"

#include <frc/Servo.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <networktables/NetworkTable.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace Modules
{
    class ShooterModule
    {
        public:
            enum State
            {
                Idle = 0,
                Shoot = 1,
                Pass = 2,
            };
        private:

            frc::Servo hoodServo{1};
            ctre::phoenix6::hardware::TalonFX* shooterMotor;

            State currentState;

            // Distance (meters): Power: HoodAngle
            Core::PiecewiseLinearFunctionXYZ shooingDistanceTable;

            double speedModifier = 1;
            
            // READ ONLY
            nt::NetworkTableEntry currentFlywheelPowerEntry;
            nt::NetworkTableEntry currentFlywheelRPMEntry;

            // READ WRITE
            nt::NetworkTableEntry targetServoPositionEntry;
            nt::NetworkTableEntry targetServoAngleEntry;

            // WRITE ONLY
            nt::NetworkTableEntry flywheelSpeedModifierEntery;
            nt::NetworkTableEntry maxAngleEntery;
            nt::NetworkTableEntry minAngleEntery;
            nt::NetworkTableEntry maxPosEntery;
            nt::NetworkTableEntry minPosEntery;
            nt::NetworkTableEntry servoOffsetEntery;

            nt::NetworkTableEntry useShooterValuesEntery;

        public:

            ShooterModule();
            ~ShooterModule();

            void ShootAtDistance(float distance);
            void Stop();
            void PassBall();

            State GetState();

            void MoveHood(float angle);
        private:
            
        
    };
}