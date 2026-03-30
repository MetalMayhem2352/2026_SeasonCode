#include "Modules/ShooterModule.h"
#include "Constants.h"

namespace Modules
{
    ShooterModule::ShooterModule()
    {
        shooterMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Shooter::SHOOTER_ID, Constants::CANIVOUR_NAME);

        shooterMotor->GetConfigurator().Apply(Constants::Shooter::shooterMotorCondiguration);

        shooingDistanceTable.LoadFromFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);
    }
    ShooterModule::~ShooterModule()
    {
        delete(shooterMotor);
    }

    void ShooterModule::ShootAtDistance(float distance)
    {
        Core::PiecewiseLinearFunctionXYZ::Output resualt = shooingDistanceTable.Get(distance);

        shooterMotor->Set(resualt.y);
        MoveHood(resualt.z);
        
        currentState = State::Shoot;
    }
    void ShooterModule::Stop()
    {
        shooterMotor->Set(0);
        currentState = Idle;
    }
    void ShooterModule::PassBall()
    {
        MoveHood(Constants::Shooter::HOOD_MAX_UP_ANGLE);
        shooterMotor->Set(1);
        currentState = Pass;
    }

    void ShooterModule::MoveHood(float angle)
    {
        angle = std::clamp<float>(angle, Constants::Shooter::HOOD_MIN_DOWN_ANGLE, Constants::Shooter::HOOD_MAX_UP_ANGLE);
        hoodServo.Set(Constants::Shooter::HOOD_TABLE.Get(angle));
    }


    ShooterModule::State ShooterModule::GetState()
    {
        return currentState;
    }

}