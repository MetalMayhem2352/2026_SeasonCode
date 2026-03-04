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

    void ShooterModule::ShootAtDistance(double distance)
    {
        currentState = State::Shoot;
        hoodMotor->Set(-0.05);
    }
    void ShooterModule::Stop()
    {
        currentState = Idle;
        hoodMotor->Set(0);
    }

    ShooterModule::State ShooterModule::GetState()
    {
        return currentState;
    }
}