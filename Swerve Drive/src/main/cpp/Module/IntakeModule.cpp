
#include "Modules/IntakeModule.h"
#include <iostream>

namespace Modules
{
    IntakeModule::IntakeModule()
    {
        leftPivotMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::LEFT_PIVOT_ID, Constants::CANIVOUR_NAME);
        rightPivotMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::RIGHT_PIVOT_ID, Constants::CANIVOUR_NAME);

        frontIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::FRONT_INTAKE_ID, Constants::CANIVOUR_NAME);
        backIntakeMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::BACK_INTAKE_ID, Constants::CANIVOUR_NAME);
        basketMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Intake::BASKET_INTAKE_ID, Constants::CANIVOUR_NAME);
        
        frontIntakeMotor->GetConfigurator().Apply(Constants::Intake::frontIntakeMotorConfig);
        backIntakeMotor->GetConfigurator().Apply(Constants::Intake::backIntakeMotorConfig);
        basketMotor->GetConfigurator().Apply(Constants::Intake::basketIntakeMotorConfig);

        pivotPIDTimer = new Core::Timer();
        pivotPIDController = new Core::PIDController(Constants::Intake::PIVOT_PID_CONFIG);
    }

    IntakeModule::~IntakeModule()
    {
        delete(leftPivotMotor);
        delete(rightPivotMotor);

        delete(frontIntakeMotor);
        delete(backIntakeMotor);
        delete(basketMotor);

        delete(pivotPIDTimer);
        delete(pivotPIDController);
    }

    void IntakeModule::Update()
    {
        pivotPIDTimer->Update();

        if (currentPivotState == Up)
        {
            targetPivotPosition = Constants::Intake::UP_ENCODER_POSITION;
        }
        else if (currentPivotState == Down)
        {
            targetPivotPosition = Constants::Intake::DOWN_ENCODER_POSITION;
        }
        else if (currentPivotState == Half)
        {
            targetPivotPosition = Constants::Intake::HALF_ENCODER_POSITION;
        }
        else
        {
            targetPivotPosition = -1; // Idle
        }
    

        if (targetPivotPosition != -1)
        {
            double pivotPower = pivotPIDController->Calculate(pivotEncoder.Get(), targetPivotPosition, pivotPIDTimer->GetDeltaTime());

            leftPivotMotor->Set(-pivotPower);
            rightPivotMotor->Set(pivotPower);
        }
        else
        {
            leftPivotMotor->Set(0);
            rightPivotMotor->Set(0);
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
                backIntakeMotor->Set(0);
                basketMotor->Set(0);
                
                break;
            }
            case State::Intaking:
            {
                frontIntakeMotor->Set(1);
                backIntakeMotor->Set(1);
                basketMotor->Set(1);
                
                break;
            }
            case State::Shooting:
            {
                frontIntakeMotor->Set(0);
                backIntakeMotor->Set(0);
                basketMotor->Set(0);

                break;
            }
            case State::Outaking:
            {
                frontIntakeMotor->Set(-0.5);
                backIntakeMotor->Set(-0.5);
                basketMotor->Set(0);

                break;
            }
        }   
    }
    IntakeModule::State IntakeModule::GetState()
    {
        return currentState;
    }
    

    void IntakeModule::SetPivot(PivotState pivotState)
    {
        currentPivotState = pivotState;
    }
    IntakeModule::PivotState IntakeModule::GetPivotState()
    {
        return currentPivotState;
    }

}