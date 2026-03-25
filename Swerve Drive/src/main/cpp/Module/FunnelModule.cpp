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

    void FunnelModule::Feed()
    {
        funnelMotor->Set(1);
    }

    void FunnelModule::Unjam()
    {
        funnelMotor->Set(-0.5);
    }
    void FunnelModule::Idle()
    {
        funnelMotor->Set(0);
    }
}