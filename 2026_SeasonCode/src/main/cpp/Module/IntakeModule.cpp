#include "Modules/IntakeModule.h"

namespace Modules
{
    IntakeModule::IntakeModule()
    {
        topIntakeMotor = new ctre::phoenix6::hardware::TalonFX(18, "Default Name");
        basketIntakeMotor = new ctre::phoenix6::hardware::TalonFX(11, "Default Name");
        basketIntakeMotor = new ctre::phoenix6::hardware::TalonFX(12, "Default Name");
        
        topIntakeMotor->GetConfigurator().Apply(Constants::Turret::shooterMotor1);
        basketIntakeMotor->GetConfigurator().Apply(Constants::Turret::shooterMotor2);
        basketIntakeMotor->GetConfigurator().Apply(Constants::Turret::shooterMotor2);
    }

    IntakeModule::~IntakeModule()
    {
        
    }

    void IntakeModule::Update()
    {
        
    }

    void IntakeModule::UpdateState(State newState)
    {
        if (newState == IntakeModule::State::Intaking)
        {
            topIntakeMotor->Set(0.5);
            basketIntakeMotor->Set(.5);
            basketIntakeMotor->Set(.5);
        }
        else if (newState == IntakeModule::State::Outaking)
        {
            topIntakeMotor->Set(-0.5);
            basketIntakeMotor->Set(-0.5);
            basketIntakeMotor->Set(-0.5);
        }
        else
        {
            topIntakeMotor->Set(0);
            basketIntakeMotor->Set(0);
        }        
    }
}