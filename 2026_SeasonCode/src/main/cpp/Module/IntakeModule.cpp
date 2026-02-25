#include "Modules/IntakeModule.h"

namespace Modules
{
    IntakeModule::IntakeModule()
    {
        topIntakeMotor = new ctre::phoenix6::hardware::TalonFX(18, "Default Name");
        basketIntakeMotor = new ctre::phoenix6::hardware::TalonFX(11, "Default Name");
        groundIntakeMotor = new ctre::phoenix6::hardware::TalonFX(-1, "Default Name");
        intakePivot = new ctre::phoenix6::hardware::TalonFX(-1, "Default Name");

        pivotPIDTimer = new Core::Timer();
        pivotPIDController = new Core::PIDController(Constants::Inake::pivotPIDConfig);
        
        topIntakeMotor->GetConfigurator().Apply(Constants::Turret::shooterMotor1);
        basketIntakeMotor->GetConfigurator().Apply(Constants::Turret::shooterMotor2);
        groundIntakeMotor->GetConfigurator().Apply(Constants::Turret::shooterMotor2);
        intakePivot->GetConfigurator().Apply(Constants::Turret::shooterMotor2);
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
        intakePivot->Set(pivotPIDController->Calculate(intakePivot->GetPosition().GetValue().value(), targetPivotPos, pivotPIDTimer->GetDeltaTime()));
    }

    void IntakeModule::UpdateState(State newState)
    {
        currentState = newState;
        switch (currentState)
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

            targetPivotPos = Constants::Inake::GROUND_PIVOT_POSITION;
            break;
        }
        case State::Outaking:
        {
            topIntakeMotor->Set(-0.5);
            basketIntakeMotor->Set(-0.5);
            groundIntakeMotor->Set(-0.5);

            targetPivotPos = Constants::Inake::GROUND_PIVOT_POSITION;
            break;
        }
        case State::Shooting:
        {
            topIntakeMotor->Set(0.5);
            basketIntakeMotor->Set(-.5);
            groundIntakeMotor->Set(-.5);
            
            targetPivotPos = Constants::Inake::SHOOT_PIVOT_POSITION;
            break;
        }
        default:
            break;
        }   
    }
}