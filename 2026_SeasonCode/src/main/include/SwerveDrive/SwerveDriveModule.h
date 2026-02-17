#pragma once

#include "SwerveDriveOdometry.h"
#include "SwervePod.h"
#include "Constants.h"
#include "Core/Timer.h"

#include <ctre/phoenix6/Pigeon2.hpp>

namespace SwerveDrive
{
    class SwerveDriveModule
    {
    private:
        // SwerveDriveOdometry SwerveDriveOdometry;

        SwervePod* frontRightPod;
        SwervePod* frontLeftPod;
        SwervePod* backLeftPod;
        SwervePod* backRightPod;
        
        Core::Timer* turningPIDTimer;
        Core::PIDController* turningPIDController;
        // Allows for more percise movement when moving and turning at hte same time
        Core::PIDController* turningWhileMovingPIDController;

        ctre::phoenix6::hardware::Pigeon2* pigeon;

        void Move(double x, double z, double yRotation);

        /// @brief Degrees
        /// @return The YAW of the giro in degrees.
        double GetYaw();
    public:
        SwerveDriveModule();
        ~SwerveDriveModule();

        void ResetIMU();
        void MoveRobotCentric(double x, double z, double yRotation);
        void MoveFieldCentric1(double x, double z, double yRotation);
        void MoveFieldCentric2(double xMovement, double zMovement, double headingDegrees);
        void Update();
    };
    
}