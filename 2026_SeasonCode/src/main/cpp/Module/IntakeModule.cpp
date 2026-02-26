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


        pivotPIDTimer = new Core::Timer();
        pivotPIDController = new Core::PIDController(Constants::Intake::pivotPIDConfig);
        
        topIntakeMotor->GetConfigurator().Apply(Constants::Intake::topIntakeMotorConfig);
        basketIntakeMotor->GetConfigurator().Apply(Constants::Intake::basketIntakeMotorConfig);
        groundIntakeMotor->GetConfigurator().Apply(Constants::Intake::groundIntakeMotorConfig);
        intakePivot->GetConfigurator().Apply(Constants::Intake::intakePivotMotorConfig);
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
        //pivotPIDTimer->Update();
        //intakePivot->Set(pivotPIDController->Calculate(intakePivot->GetPosition().GetValue().value(), targetPivotPos, pivotPIDTimer->GetDeltaTime()));
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
            groundIntakeMotor->Set(0);
            break;
        }
        case State::Intaking:
        {
            topIntakeMotor->Set(0.5);
            basketIntakeMotor->Set(.5);
            groundIntakeMotor->Set(.5);

            targetPivotPos = Constants::Intake::GROUND_PIVOT_POSITION;
            break;
        }
        case State::Outaking:
        {
            topIntakeMotor->Set(-0.5);
            basketIntakeMotor->Set(-0.5);
            groundIntakeMotor->Set(-0.5);

            targetPivotPos = Constants::Intake::GROUND_PIVOT_POSITION;
            break;
        }
        case State::Shooting:
        {
            topIntakeMotor->Set(0.5);
            basketIntakeMotor->Set(-.5);
            groundIntakeMotor->Set(-.5);
            
            targetPivotPos = Constants::Intake::SHOOT_PIVOT_POSITION;
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