#include "Pathing/SwerveDrive.h"
#include "Pathing/TunerConstants.h"

namespace Pathing
{
    SwerveDrive::SwerveDrive()
    {
        swerveDrive = new ctre::phoenix6::swerve::SwerveDrivetrain<ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::CANcoder>
                (TunerConstants::DrivetrainConstants, TunerConstants::FrontLeft, TunerConstants::FrontRight, TunerConstants::BackLeft, TunerConstants::BackRight);
    }
    
    SwerveDrive::~SwerveDrive()
    {
        
    }
    
    void SwerveDrive::Update()
    {
        
    }
    
    void SwerveDrive::Move(double x, double z, double rotation)
    {
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            x * TunerConstants::kMaxSpeed,
            z * TunerConstants::kMaxSpeed,
            rotation * TunerConstants::kMaxAngularSpeed,
            swerveDrive->GetRotation3d().ToRotation2d()
        );

        ctre::phoenix6::swerve::requests::FieldCentric fieldCentric;
        
        fieldCentric.WithVelocityX(speeds.vx).WithVelocityY(speeds.vy).WithRotationalRate(speeds.omega);


        swerveDrive->SetControl(fieldCentric);
    }
}