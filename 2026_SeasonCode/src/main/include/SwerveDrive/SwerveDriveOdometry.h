#pragma once

#include "SwervePod.h"
#include "RobotPosition.h"

#include <ctre/phoenix6/Pigeon2.hpp>


namespace SwerveDrive
{
    // TODO: Make Base Odometry Class to inherite from
    class SwerveDriveOdometry 
    {
    private:
        SwerveDrive::SwervePod& frontRightPod;
        SwerveDrive::SwervePod& frontLeftPod;
        SwerveDrive::SwervePod& backRightPod;
        SwerveDrive::SwervePod& backLeftPod;
        ctre::phoenix6::hardware::Pigeon2& pigeon;

        double lastHeading;

        DriveBase::RobotPosition currentRobotPosition;
        DriveBase::RobotPosition offsetPosition;
        
    public:
        SwerveDriveOdometry(SwerveDrive::SwervePod& frontRight, SwerveDrive::SwervePod& frontLeft, SwerveDrive::SwervePod& backRight, SwerveDrive::SwervePod& backLeft, ctre::phoenix6::hardware::Pigeon2& pigeon);
        ~SwerveDriveOdometry();

        void Update();

        void SetStartPosition(DriveBase::RobotPosition startPosition);
        void ResetPosition();

        DriveBase::RobotPosition GetRobotPosition();
    };
}
