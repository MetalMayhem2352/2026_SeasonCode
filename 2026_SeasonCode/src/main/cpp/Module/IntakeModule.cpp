
#include "Modules/IntakeModule.h"

namespace Modules
{
    IntakeModule::IntakeModule()
    {
        frontIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::TOP_INTAKE_ID, Constants::CANIVOUR_NAME);
        basketIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::BASKET_INTAKE_ID, Constants::CANIVOUR_NAME);
        bottomIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::GROUND_INTAKE_ID, Constants::CANIVOUR_NAME);
        intakePivot = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::PIVOT_ID, Constants::CANIVOUR_NAME);
        intakePivot2 = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::PIVOT_ID2, Constants::CANIVOUR_NAME);

        frontIntakeMotor->GetConfigurator().Apply(Constants::Intake::topIntakeMotorConfig);
        basketIntakeMotor->GetConfigurator().Apply(Constants::Intake::basketIntakeMotorConfig);
        bottomIntakeMotor->GetConfigurator().Apply(Constants::Intake::groundIntakeMotorConfig);
        intakePivot->GetConfigurator().Apply(Constants::Intake::intakePivotMotorConfig);
        intakePivot2->GetConfigurator().Apply(Constants::Intake::intakePivotMotor2Config);

        intakePivot->SetPosition(units::angle::turn_t(0));

        intakePivotPos = intakePivot->GetPosition().GetValue().value();;
        targetPivotPos = 0;

        pivotPIDTimer = new Core::Timer();
        pivotPIDController = new Core::PIDController(Constants::Intake::PivotPIDConfig);
    }

    IntakeModule::~IntakeModule()
    {
        delete(frontIntakeMotor);
        delete(basketIntakeMotor);
        delete(bottomIntakeMotor);
        delete(intakePivot);
        delete(intakePivot2);
        delete(pivotPIDTimer);
        delete(pivotPIDController);
    }

    void IntakeModule::Update()
    {
        pivotPIDTimer->Update();

        intakePivot->Set(pivotPIDController->Calculate(intakePivotPos, targetPivotPos, pivotPIDTimer->GetDeltaTime()));
        intakePivot2->Set(pivotPIDController->Calculate(intakePivotPos, targetPivotPos, pivotPIDTimer->GetDeltaTime()));

        if (currentState == Shooting)
        {
            if (pivotPIDTimer->GetStartTime() < 2.5)
            {
                frontIntakeMotor->Set(1);
                basketIntakeMotor->Set(-1);
            }
            else if (pivotPIDTimer->GetStartTime() < 3)
            {
                frontIntakeMotor->Set(-1);
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
            frontIntakeMotor->Set(0);
            bottomIntakeMotor->Set(0);
            basketIntakeMotor->Set(0);

            targetPivotPos = 0;
            break;
        }
        case State::Intaking:
        {
            frontIntakeMotor->Set(0.5);
            bottomIntakeMotor->Set(0.5);
            basketIntakeMotor->Set(0.5);
            targetPivotPos == 0; // intake down pos set
            break;
        }
        case State::Shooting:
        {
            
            frontIntakeMotor->Set(0.5);
            bottomIntakeMotor->Set(0.5);
            basketIntakeMotor->Set(0.5);
            break;
        }
        case State::intake_up:
        {
            targetPivotPos == 0; // set intake down position here TODO
            break;
        }
        case State::Unjamming:
        {
            basketIntakeMotor->Set(-0.5);
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