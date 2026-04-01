#include "Pathing/CTRESwerveDrive.h"
#include "Pathing/TunerConstants.h"

namespace Pathing
{
    CTRESwerveDrive::CTRESwerveDrive()
    {
        odometry = new Odometry(&swerveDrive, new ctre::phoenix6::hardware::Pigeon2(Constants::pigeonID, Constants::CANIVOUR_NAME));
    }
    
    CTRESwerveDrive::~CTRESwerveDrive()
    {
        
    }
    
    void CTRESwerveDrive::Update()
    {
        odometry->Update();
    }
    
    void CTRESwerveDrive::Move(double x, double z, double rotation)
    {
        
        swerveDrive.SetControl(m_driveRequest.WithVelocityX(z * 0.25 * TunerConstants::kSpeedAt12Volts).WithVelocityY(x * 0.25 * TunerConstants::kSpeedAt12Volts).WithRotationalRate(rotation * 0.25 * TunerConstants::kRotationSpeedAt12Volts));
    }
    
    void CTRESwerveDrive::ResetYaw()
    {
        swerveDrive.SeedFieldCentric();
    }

    Odometry* CTRESwerveDrive::GetOdometery()
    {
        return odometry;
    }
}