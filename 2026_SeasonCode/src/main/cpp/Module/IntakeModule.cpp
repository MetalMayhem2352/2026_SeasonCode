#include "Modules/IntakeModule.h"

namespace Modules
{
    IntakeModule::IntakeModule()
    {
        topIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::TOP_INTAKE_ID, Constants::CANIVOUR_NAME);
        basketIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::BASKET_INTAKE_ID, Constants::CANIVOUR_NAME);
        groundIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::GROUND_INTAKE_ID, Constants::CANIVOUR_NAME);
        intakePivot = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::PIVOT_ID, Constants::CANIVOUR_NAME);

        topIntakeMotor->GetConfigurator().Apply(Constants::Intake::topIntakeMotorConfig);
        basketIntakeMotor->GetConfigurator().Apply(Constants::Intake::basketIntakeMotorConfig);
        groundIntakeMotor->GetConfigurator().Apply(Constants::Intake::groundIntakeMotorConfig);
        intakePivot->GetConfigurator().Apply(Constants::Intake::intakePivotMotorConfig);

        intakePivot->SetPosition(units::angle::turn_t(0));


        pivotPIDTimer = new Core::Timer();
        pivotPIDController = new Core::PIDController(Constants::Intake::PivotPIDConfig);
    }

    IntakeModule::~IntakeModule()
    {
        delete(topIntakeMotor);
        delete(basketIntakeMotor);
        delete(groundIntakeMotor);
        delete(intakePivot);
        
        delete(pivotPIDTimer);
        delete(pivotPIDController);
    }

    void IntakeModule::Update()
    {
        pivotPIDTimer->Update();

        if (targetPivotPos == 1)
        {
            intakePivot->Set(-0.5);
        }
        else
        {
            intakePivot->Set(0);
        }

        if (currentState == Shooting)
        {
            if (pivotPIDTimer->GetStartTime() < 2.5)
            {
                topIntakeMotor->Set(1);
                basketIntakeMotor->Set(-1);
            }
            else if (pivotPIDTimer->GetStartTime() < 3)
            {
                topIntakeMotor->Set(-1);
                basketIntakeMotor->Set(1);
            }
            else
            {
                pivotPIDTimer->Reset();
            }

        }

        if (targetPivotPos == 0)
        {
            intakePivot->Set(0);
        }
        else
        {
            intakePivot->Set(-0.25);
        }
    }

    void IntakeModule::UpdateState(State newState)
    {
        currentState = newState;
        switch (newState)
        {
        case State::Idle:
        {
            topIntakeMotor->Set(0);
            basketIntakeMotor->Set(0);
            // groundIntakeMotor->Set(0);

            targetPivotPos = 0;
            break;
        }
        case State::Intaking:
        {
            topIntakeMotor->Set(0.5);
            basketIntakeMotor->Set(1);
            // groundIntakeMotor->Set(0.5);

            targetPivotPos = 0;
            break;
        }
        case State::Shooting:
        {
            topIntakeMotor->Set(1);
            basketIntakeMotor->Set(-1);
            // groundIntakeMotor->Set(-1);
            
            targetPivotPos = 1;
            break;
        }
        case State::Outaking:
        {
            topIntakeMotor->Set(-1);
            basketIntakeMotor->Set(-1);
            // groundIntakeMotor->Set(-1);
            
            targetPivotPos = 0;
            break;
        }
        case State::Unjamming:
        {
            topIntakeMotor->Set(-1);
            basketIntakeMotor->Set(1);
            // groundIntakeMotor->Set(-1);
            
            targetPivotPos = 1;
            break;
        }
        case State::GroundShoot:
        {
            topIntakeMotor->Set(1);
            basketIntakeMotor->Set(-1);
            // groundIntakeMotor->Set(-1);
            
            targetPivotPos = 0;
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