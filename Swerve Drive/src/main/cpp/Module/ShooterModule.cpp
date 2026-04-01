#include "Modules/ShooterModule.h"
#include "Constants.h"
#include <iostream>

namespace Modules
{
    ShooterModule::ShooterModule()
    {
        shooterMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Shooter::SHOOTER_ID, Constants::CANIVOUR_NAME);

        shooterMotor->GetConfigurator().Apply(Constants::Shooter::shooterMotorCondiguration);

        shooingDistanceTable.LoadFromFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);

        std::cout << "\n\n\n########## Distance Table ##########\nx: 0, y:" << shooingDistanceTable.Get(0).y << ", z:" << shooingDistanceTable.Get(0).z << "\nx: 1, y: "
            << shooingDistanceTable.Get(20).y << ", z:" << shooingDistanceTable.Get(20).z << "\n\n";
    }
    ShooterModule::~ShooterModule()
    {
        delete(shooterMotor);
    }
    bool i;
    int j = 0;
    void ShooterModule::ShootAtDistance(float distance)
    {
        j++;

        shooingDistanceTable.LoadFromFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);

        Core::PiecewiseLinearFunctionXYZ::Output resualt = shooingDistanceTable.Get(distance);

        if (i == false)
        {
            i = true;
            std::cout << "Power: " << resualt.y << "; Hood: " << resualt.z << '\n';
        }
        MoveHood(resualt.z);
        
        shooterMotor->Set(resualt.y);
        currentState = State::Shoot;
    }
    void ShooterModule::Stop()
    {
        i = false;
        // shooingDistanceTable.SaveToFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);

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
        
        double hoodPos = Constants::Shooter::HOOD_TABLE.Get(angle);
        
        std::cout << "Hood Angle: " << hoodPos << "\n";

        hoodServo.Set(hoodPos);

    }


    ShooterModule::State ShooterModule::GetState()
    {
        return currentState;
    }

}