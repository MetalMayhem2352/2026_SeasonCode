#include <frc/Timer.h>
#include "Modules/TurretFeeder.h"
#include "Constants.h"

TurretFeeder::TurretFeeder()
{
    feederMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Feeder::feederMotor_ID, Constants::CANIVOUR_NAME);
}
void TurretFeeder::Feed()
{
    feederMotor->Set(0.5);
}
void TurretFeeder::Unjam()
{
    feederMotor->Set(-0.5);
}
void TurretFeeder::Idle()
{
    feederMotor->Set(0);
}
