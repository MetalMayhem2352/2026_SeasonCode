#include "Modules/TestModule.h"

namespace Modules
{
    TestModule::TestModule()
    {
        flywheelMotor1 = new ctre::phoenix6::hardware::TalonFX(15, "Default Name");
        flywheelMotor2 = new ctre::phoenix6::hardware::TalonFX(14, "Default Name");

        flywheelMotor1->GetConfigurator().Apply(Constants::Turret::shooterMotor1);
        flywheelMotor2->GetConfigurator().Apply(Constants::Turret::shooterMotor2);
    }

    TestModule::~TestModule()
    {
        delete(flywheelMotor1);
        delete(flywheelMotor2);
    }

    void TestModule::Shoot()
    {
        flywheelMotor1->Set(0.7);
        flywheelMotor2->Set(0.7);
    }

    void TestModule::Stop()
    {
        flywheelMotor1->Set(0);
        //flywheelMotor2->Set(0);
    }
}