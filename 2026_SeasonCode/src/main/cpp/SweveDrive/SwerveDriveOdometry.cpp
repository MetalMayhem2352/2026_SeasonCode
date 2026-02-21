#include "SwerveDrive/SwerveDriveOdometry.h"
#include "Constants.h"

// TODO: Make Base Odometry Class to inherite from
namespace SwerveDrive
{
    SwerveDriveOdometry::SwerveDriveOdometry(   SwerveDrive::SwervePod& frontRight, SwerveDrive::SwervePod& frontLeft, 
                                                SwerveDrive::SwervePod& backRight, SwerveDrive::SwervePod& backLeft, 
                                                ctre::phoenix6::hardware::Pigeon2& pigeon)
        : frontRightPod(frontRight), frontLeftPod(frontLeft), backRightPod(backRight), backLeftPod(backLeft), pigeon(pigeon)
    {
        pigeon.Reset();
    }
    
    SwerveDriveOdometry::~SwerveDriveOdometry()
    {

    }


    void SwerveDriveOdometry::Update()
    {
        bool canNormalize = false;
        double currentHeading = pigeon.GetYaw().GetValue().value();
        if (currentHeading > 360 || currentHeading <= 0)
        {

        }
        currentHeading = NormalizeDeg0360(currentHeading);
        double headingDelta = currentHeading - lastHeading;
        
        
        Core::Vector2 robotMovementDelta = GetMovementDelta(headingDelta);


        
        double averageOrientation = (currentHeading - (headingDelta / 2));
        double averageOrientationRad = averageOrientation * Constants::DEGREES_TO_RADIANS;

        Core::Vector2 fieldMovementDelta((robotMovementDelta.GetX() * std::cos(averageOrientationRad)) - (robotMovementDelta.GetY() * std::sin(averageOrientationRad)), 
                                        (robotMovementDelta.GetX() * std::sin(averageOrientationRad)) + (robotMovementDelta.GetY() * std::cos(averageOrientationRad)));

        currentRobotPosition += fieldMovementDelta;
        currentRobotPosition.heading = currentHeading;

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


    Core::Vector2 SwerveDriveOdometry::GetMovementDelta(double headingDelta)
    {
        Core::Vector2 fl_nonRotationMovementDelta = GetPodMovementDelta(headingDelta, Constants::Swerve::FrontRightPod::TURN_VECTOR, frontRightPod);
        Core::Vector2 fr_nonRotationMovementDelta = GetPodMovementDelta(headingDelta, Constants::Swerve::FrontRightPod::TURN_VECTOR, frontRightPod);
        Core::Vector2 br_nonRotationMovementDelta = GetPodMovementDelta(headingDelta, Constants::Swerve::BackRightPod::TURN_VECTOR, backRightPod);
        Core::Vector2 bl_nonRotationMovementDelta = GetPodMovementDelta(headingDelta, Constants::Swerve::BackLeftPod::TURN_VECTOR, backLeftPod);

        // Get average if the movement Vectors
        Core::Vector2 average = (fl_nonRotationMovementDelta + fr_nonRotationMovementDelta + br_nonRotationMovementDelta + bl_nonRotationMovementDelta) / 4;

        return average;
    }

    
    double SwerveDriveOdometry::NormalizeDeg0360(double deg)
    {
        while (deg <= 0) 
        {
            deg += 360.0;
        }
        while (deg > 360.0) 
        {
            deg -= 360.0;
        }
        return deg;
    }
    
    Core::Vector2 SwerveDriveOdometry::GetPodMovementDelta(double robotHeadingDelta, Core::Vector2 turnVector, SwervePod& pod)
    {
        double rotaionDistanceDelta = (Constants::Swerve::DIAGONAL_MODULE_OFFSET * robotHeadingDelta * std::numbers::pi) / 180;
        Core::Vector2 rotaionDelta = turnVector * rotaionDistanceDelta;
        
        Core::Vector2 podMovementDelta = Core::Vector2::CreateAngularVector(pod.GetAngle() - (pod.GetAngleDelta() / 2), pod.GetMovementDelta() * Constants::Swerve::METERS_PER_TICK);
        return podMovementDelta - rotaionDelta;
    }


}