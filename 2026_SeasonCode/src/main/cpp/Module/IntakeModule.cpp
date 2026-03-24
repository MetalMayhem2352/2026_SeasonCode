
#include "Modules/IntakeModule.h"

namespace Modules
{
    IntakeModule::IntakeModule()
    {
        // frontIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::FRONT_INTAKE_ID, Constants::CANIVOUR_NAME);
        // backIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::BACK_INTAKE_ID, Constants::CANIVOUR_NAME);
        basketMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::BASKET_INTAKE_ID, Constants::CANIVOUR_NAME);
        
        // frontIntakeMotor->GetConfigurator().Apply(Constants::Intake::frontIntakeMotorConfig);
        // backIntakeMotor->GetConfigurator().Apply(Constants::Intake::backIntakeMotorConfig);
        basketMotor->GetConfigurator().Apply(Constants::Intake::basketIntakeMotorConfig);
    }

    IntakeModule::~IntakeModule()
    {
        // delete(frontIntakeMotor);
        // delete(backIntakeMotor);
        delete(basketMotor);
    }

    void IntakeModule::Update()
    {
      
    }

    void IntakeModule::UpdateState(State newState)
    {
        currentState = newState;
        switch (newState)
        {
        case State::Idle:
        {
            // frontIntakeMotor->Set(0);
            // backIntakeMotor->Set(0);
            basketMotor->Set(0);

            break;
        }
        case State::Intaking:
        {
            // frontIntakeMotor->Set(0.5);
            // backIntakeMotor->Set(0.5);
            basketMotor->Set(1);
            break;
        }
        case State::Shooting:
        {
            // frontIntakeMotor->Set(-0.5);
            // backIntakeMotor->Set(-0.5);
            basketMotor->Set(0);
            break;
        }
        case State::Outaking:
        {
            // frontIntakeMotor->Set(-0.5);
            // backIntakeMotor->Set(-0.5);
            basketMotor->Set(0);
            break;
        }
        default:
            break;
        }   
    }

    IntakeModule::State IntakeModule::GetState()
    {
        return currentState;
    }
}