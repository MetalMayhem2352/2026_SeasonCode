
#include "Modules/Basket.h"
#include "Core/Timer.h"
#include "Core/PIDController.h"
#include "Constants.h"

#include <core/LimelightHelpers.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>

namespace Modules
{
    Basket::Basket()
    {
        leftBasketMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Basket::LEFT_BASKET_ID, Constants::CANIVOUR_NAME);
        rightBasketMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Basket::RIGHT_BASKET_ID, Constants::CANIVOUR_NAME);
        
        leftBasketMotor->GetConfigurator().Apply(Constants::Basket::leftBasketMotorConfig);
        rightBasketMotor->GetConfigurator().Apply(Constants::Basket::rightBasketMotorConfig);

        pidTimer = new Core::Timer();
        leftPIDController = new Core::PIDController(Constants::Basket::basketPIDConfig);
        rightPIDController = new Core::PIDController(Constants::Basket::basketPIDConfig);
    }

    Basket::~Basket()
    {
        delete(leftBasketMotor);
        delete(rightBasketMotor);

        delete(pidTimer);
        delete(leftPIDController);
        delete(rightPIDController);
    }

    void Basket::Update()
    {
        pidTimer->Update();
        leftBasketMotor->Set(leftPIDController->Calculate(leftBasketMotor->GetPosition().GetValue().value(), targetPosition, pidTimer->GetDeltaTime()));
        rightBasketMotor->Set(rightPIDController->Calculate(rightBasketMotor->GetPosition().GetValue().value(), targetPosition, pidTimer->GetDeltaTime()));
    }
    
    void Basket::UpdateState(State newState)
    {
        switch (newState)
        {
        case State::High:
            targetPosition == Constants::Basket::UP_POSITION;
            break;
        case State::Low:
            targetPosition == Constants::Basket::DOWN_POSITON;
            break;
        default:
            break;
        }
    }
}