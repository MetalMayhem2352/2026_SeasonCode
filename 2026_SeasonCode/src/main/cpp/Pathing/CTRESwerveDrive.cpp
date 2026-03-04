#include "Pathing/CTRESwerveDrive.h"
#include "Pathing/TunerConstants.h"

namespace Pathing
{
    CTRESwerveDrive::CTRESwerveDrive()
    {
        
    }
    
    CTRESwerveDrive::~CTRESwerveDrive()
    {
        
    }
    
    void CTRESwerveDrive::Update()
    {
        
    }
    
    void CTRESwerveDrive::Move(double x, double z, double rotation)
    {
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            x * TunerConstants::kMaxSpeed,
            z * TunerConstants::kMaxSpeed,
            rotation * TunerConstants::kMaxAngularSpeed,
            swerveDrive.GetRotation3d().ToRotation2d()
        );

        ctre::phoenix6::swerve::requests::FieldCentric fieldCentric{};
        
        fieldCentric.WithVelocityX(speeds.vx).WithVelocityY(speeds.vy).WithRotationalRate(speeds.omega);

        std::cout << "swerve drive : " << fieldCentric.RotationalRate.value() << "\n";
         
        std::cout << "swerveDrive->GteModules : " << swerveDrive.GetModules()[0] << "\n";
        swerveDrive.SetControl(fieldCentric);

        //

    }
}