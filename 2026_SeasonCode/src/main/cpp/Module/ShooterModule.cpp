#include "Modules/ShooterModule.h"
#include "Constants.h"

namespace Modules
{
    ShooterModule::ShooterModule()
    {
        shooterMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Shooter::shooterID, Constants::CANIVOUR_NAME);
    }
    ShooterModule::~ShooterModule()
    {
    }

    void ShooterModule::ShootAtDistance(double distance)
    {
        shooterMotor->Set(1);
        currentState = State::Shoot;
    }
    void ShooterModule::Stop()
    {
        currentState = Idle;
        shooterMotor->Set(0);
    }


    ShooterModule::State ShooterModule::GetState()
    {
        return currentState;
    }

}