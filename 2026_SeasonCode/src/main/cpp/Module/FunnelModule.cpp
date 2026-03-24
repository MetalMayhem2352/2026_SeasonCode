#include <frc/Timer.h>
#include "Modules/FunnelModule.h"
#include "Constants.h"

FunnelModule::FunnelModule()
{
    funnelMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Funnel::Funnel_MOTOR_ID, Constants::CANIVOUR_NAME);
}
void FunnelModule::Feed()
{
    funnelMotor->Set(0.5);
}
void FunnelModule::Unjam()
{
    funnelMotor->Set(-0.5);
}
void FunnelModule::Idle()
{
    funnelMotor->Set(0);
}
