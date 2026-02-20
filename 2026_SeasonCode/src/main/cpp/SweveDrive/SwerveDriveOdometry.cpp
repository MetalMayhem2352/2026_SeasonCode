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

        // ------------------ MIGHT not need to rotate movement delta to be field centric ----------------------------- 
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
        Core::Vector2 fl_nonRotationMovementDelta;
        Core::Vector2 fr_nonRotationMovementDelta;
        Core::Vector2 br_nonRotationMovementDelta;
        Core::Vector2 bl_nonRotationMovementDelta;
        if (headingDelta != 0)
        {
            double fl_rotaionDistanceDelta = (Constants::Swerve::FrontLeftPod::MODULE_DISTANCE_FROM_PIVOT * headingDelta * std::numbers::pi) / 180;
            Core::Vector2 fl_RoationDelta = Constants::Swerve::FrontLeftPod::TURN_VECTOR * fl_rotaionDistanceDelta;
            Core::Vector2 fl_podMovementDelta = Core::Vector2::CreateAngularVector(frontLeftPod.GetAngle(), frontLeftPod.GetMovementDelta());
            fl_nonRotationMovementDelta = fl_podMovementDelta - fl_RoationDelta;

            double fr_rotaionDistanceDelta = (Constants::Swerve::FrontRightPod::MODULE_DISTANCE_FROM_PIVOT * headingDelta * std::numbers::pi) / 180;
            Core::Vector2 fr_RoationDelta = Constants::Swerve::FrontRightPod::TURN_VECTOR * fr_rotaionDistanceDelta;
            Core::Vector2 fr_podMovementDelta = Core::Vector2::CreateAngularVector(frontRightPod.GetAngle(), frontRightPod.GetMovementDelta());
            fr_nonRotationMovementDelta = fr_podMovementDelta - fr_RoationDelta;
            
            double br_rotaionDistanceDelta = (Constants::Swerve::BackRightPod::MODULE_DISTANCE_FROM_PIVOT * headingDelta * std::numbers::pi) / 180;
            Core::Vector2 br_RoationDelta = Constants::Swerve::BackRightPod::TURN_VECTOR * br_rotaionDistanceDelta;
            Core::Vector2 br_podMovementDelta = Core::Vector2::CreateAngularVector(backRightPod.GetAngle(), backRightPod.GetMovementDelta());
            br_nonRotationMovementDelta = br_podMovementDelta - br_RoationDelta;
            
            double bl_rotaionDistanceDelta = (Constants::Swerve::BackLeftPod::MODULE_DISTANCE_FROM_PIVOT * headingDelta * std::numbers::pi) / 180;
            Core::Vector2 bl_RoationDelta = Constants::Swerve::BackLeftPod::TURN_VECTOR * bl_rotaionDistanceDelta;
            Core::Vector2 bl_podMovementDelta = Core::Vector2::CreateAngularVector(backLeftPod.GetAngle(), backLeftPod.GetMovementDelta());
            bl_nonRotationMovementDelta = bl_podMovementDelta - bl_RoationDelta;
        }
        else
        {
            fl_nonRotationMovementDelta = Core::Vector2::CreateAngularVector(frontLeftPod.GetAngle(), frontLeftPod.GetMovementDelta());
            fr_nonRotationMovementDelta = Core::Vector2::CreateAngularVector(frontRightPod.GetAngle(), frontRightPod.GetMovementDelta());
            br_nonRotationMovementDelta = Core::Vector2::CreateAngularVector(backRightPod.GetAngle(), backRightPod.GetMovementDelta());
            bl_nonRotationMovementDelta = Core::Vector2::CreateAngularVector(backLeftPod.GetAngle(), backLeftPod.GetMovementDelta());
        }
        

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

}