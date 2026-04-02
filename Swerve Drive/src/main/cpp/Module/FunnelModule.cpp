#include <frc/Timer.h>
#include "Modules/FunnelModule.h"
#include "Constants.h"

namespace Modules
{
    FunnelModule::FunnelModule()
    {
        funnelMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Funnel::FUNNEL_MOTOR_ID, Constants::CANIVOUR_NAME);
        
        funnelMotor->GetConfigurator().Apply(Constants::Funnel::funnelMotorConfig);
    }
    FunnelModule::~FunnelModule()
    {
        delete(funnelMotor);
    }

    void FunnelModule::Update()
    {
    }

    void FunnelModule::UpdateState(State state)
    {
        currentState = state;
        switch (state)
        {
        case Idle:
            funnelMotor->Set(0);
            break;
        case Feed:
            funnelMotor->Set(0.25);
            break;
        case Unjam:
            funnelMotor->Set(-0.5);
            break;
        }
    }

    FunnelModule::State FunnelModule::GetState()
    {
        return currentState;
    }
}