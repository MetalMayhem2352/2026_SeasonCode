#pragma once

#include "SwervePod.h"
#include "RobotPosition.h"

#include <ctre/phoenix6/Pigeon2.hpp>


namespace CustomSwerveDrive
{
    // TODO: Make Base Odometry Class to inherite from
    class SwerveDriveOdometry 
    {
    private:
        SwervePod* frontRightPod;
        SwervePod* frontLeftPod;
        SwervePod* backRightPod;
        SwervePod* backLeftPod;
        ctre::phoenix6::hardware::Pigeon2* pigeon;

        double lastHeading;

        CustomDriveBase::RobotPosition currentRobotPosition;
        CustomDriveBase::RobotPosition offsetPosition;
        
    public:
        SwerveDriveOdometry(SwervePod* frontRight, SwervePod* frontLeft, SwervePod* backRight, SwervePod* backLeft, ctre::phoenix6::hardware::Pigeon2* pigeon);
        ~SwerveDriveOdometry();

        void Update();

        void SetStartPosition(CustomDriveBase::RobotPosition startPosition);
        void ResetPosition();

        CustomDriveBase::RobotPosition GetRobotPosition();

    private:
        Core::Vector2 GetMovementDelta(double headingDelta);
        static double NormalizeDeg0360(double deg);
        Core::Vector2 GetPodMovementDelta(double robotHeadingDelta, Core::Vector2 turnVector, SwervePod*& pod); 


    };
}
