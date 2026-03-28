#include "Modules/ShooterModule.h"
#include "Constants.h"

namespace Modules
{
    ShooterModule::ShooterModule()
    {
        shooterMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Shooter::shooterID, Constants::CANIVOUR_NAME);

        shooterMotor->GetConfigurator().Apply(Constants::Shooter::shooterMotorCondiguration);
    }
    ShooterModule::~ShooterModule()
    {
        delete(shooterMotor);
    }

    void ShooterModule::ShootAtDistance(double distance)
    {
        shooterMotor->Set(1);
        currentState = State::Shoot;
    }
    void ShooterModule::Stop()
    {
        currentState = Idle;
    }


    ShooterModule::State ShooterModule::GetState()
    {
        return currentState;
    }

}