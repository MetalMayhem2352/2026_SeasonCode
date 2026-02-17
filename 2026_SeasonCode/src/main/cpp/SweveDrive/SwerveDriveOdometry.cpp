#include "SwerveDrive/SwerveDriveOdometry.h"

// TODO: Make Base Odometry Class to inherite from
namespace SwerveDrive
{
    SwerveDriveOdometry::SwerveDriveOdometry(   SwerveDrive::SwervePod& frontRight, SwerveDrive::SwervePod& frontLeft, 
                                                SwerveDrive::SwervePod& backRight, SwerveDrive::SwervePod& backLeft, 
                                                ctre::phoenix6::hardware::Pigeon2& pigeon)
        : frontRightPod(frontRight), frontLeftPod(frontLeft), backRightPod(backRight), backLeftPod(backLeft), pigeon(pigeon)
    {
        
    }
    
    SwerveDriveOdometry::~SwerveDriveOdometry()
    {

    }


    void SwerveDriveOdometry::Update()
    {
        double currentHeading = pigeon.GetYaw().GetValue().value();
        double headingDelta = currentHeading - lastHeading; 

        // Use law of cosings to figure out the distance travled
        // headingDelta * 

        // Calculate Position Deltas

        lastHeading = currentHeading;
    }   

    void SwerveDriveOdometry::SetStartPosition(DriveBase::RobotPosition startPosition)
    {
        offsetPosition = startPosition;
        
    }

    void SwerveDriveOdometry::ResetPosition()
    {
        currentRobotPosition = offsetPosition;
    }

    DriveBase::RobotPosition SwerveDriveOdometry::GetRobotPosition()
    {
        return currentRobotPosition;
    }
}