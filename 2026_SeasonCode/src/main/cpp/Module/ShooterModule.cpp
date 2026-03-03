#include "Modules/ShooterModule.h"
#include "Constants.h"

namespace Modules
{
    ShooterModule::ShooterModule()
    {
        shooterMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Shooter::shooterID, Constants::CANIVOUR_NAME);
        hoodMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Shooter::hoodID, Constants::CANIVOUR_NAME);
    }
    ShooterModule::~ShooterModule()
    {
        
    }

    void ShooterModule::Shoot(Pathing::RobotPosition* const position)
    {
        hoodMotor->Set(-0.05);
    }
    void ShooterModule::Stop()
    {
        hoodMotor->Set(0);
    }
}