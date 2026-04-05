#include "Modules/ShooterModule.h"
#include "Constants.h"
#include <iostream>

namespace Modules
{
    ShooterModule::ShooterModule()
    {

        shooterMotor = new ctre::phoenix6::hardware::TalonFX(Constants::Shooter::SHOOTER_ID, Constants::CANIVOUR_NAME);

        shooterMotor->GetConfigurator().Apply(Constants::Shooter::shooterMotorCondiguration);

        shooingDistanceTable.LoadFromFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);
    

        nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
        
        inst.SetServerTeam(2352);
        inst.StartClient4("MyConsoleClient");
        
        std::shared_ptr<nt::NetworkTable> networkTable = inst.GetTable("SmartDashboard");

    
        currentFlywheelPowerEntry = networkTable->GetEntry("flywheelPower");
        currentFlywheelRPMEntry = networkTable->GetEntry("flywheelRPM");
        flywheelSpeedModifierEntery = networkTable->GetEntry("flywheelSpeedModifier");

        targetServoAngleEntry = networkTable->GetEntry("taregetServoAngle");
        maxAngleEntery = networkTable->GetEntry("servoMinAngle");
        minAngleEntery = networkTable->GetEntry("servoMaxAngle");
        maxPosEntery = networkTable->GetEntry("maxPosEntery");
        minPosEntery = networkTable->GetEntry("minPosEntery");
        servoOffsetEntery = networkTable->GetEntry("servoOffset");

        useShooterValuesEntery = networkTable->GetEntry("lockShooterValues");

        currentFlywheelPowerEntry.SetDouble(0);
        currentFlywheelRPMEntry.SetDouble(0);
        flywheelSpeedModifierEntery.SetDouble(1);

        targetServoAngleEntry.SetDouble(1);
        maxAngleEntery.SetDouble(Constants::Shooter::HOOD_MAX_UP_ANGLE);
        minAngleEntery.SetDouble(Constants::Shooter::HOOD_MIN_DOWN_ANGLE);
        maxPosEntery.SetDouble(Constants::Shooter::HOOD_MAX_UP_POS);
        minPosEntery.SetDouble(Constants::Shooter::HOOD_MIN_DOWN_POS);
        servoOffsetEntery.SetDouble(0);
        
        useShooterValuesEntery.SetBoolean(false);
    }
    ShooterModule::~ShooterModule()
    {
        delete(shooterMotor);
    }
    void ShooterModule::ShootAtDistance(float distance)
    {
        shooingDistanceTable.LoadFromFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);

        Core::PiecewiseLinearFunctionXYZ::Output resualt = shooingDistanceTable.Get(distance);

        double hoodAngle = resualt.z;
        double servoOffset = 0;
        double shooterPower = distance;
        
        
        if (useShooterValuesEntery.GetBoolean(false))
        {
            // speedModifier = flywheelSpeedModifierEntery.GetDouble(1);
            // shooterPower = currentFlywheelPowerEntry.GetDouble(shooterPower);
            
            hoodAngle = targetServoAngleEntry.SetDouble(hoodAngle);
            Constants::Shooter::HOOD_MAX_UP_ANGLE = maxAngleEntery.SetDouble(Constants::Shooter::HOOD_MAX_UP_ANGLE);
            Constants::Shooter::HOOD_MIN_DOWN_ANGLE = minAngleEntery.SetDouble(Constants::Shooter::HOOD_MIN_DOWN_ANGLE);
            Constants::Shooter::HOOD_MAX_UP_POS = maxPosEntery.SetDouble(Constants::Shooter::HOOD_MAX_UP_POS);
            Constants::Shooter::HOOD_MIN_DOWN_POS = minPosEntery.SetDouble(Constants::Shooter::HOOD_MIN_DOWN_POS);

            servoOffset = servoOffsetEntery.GetDouble(servoOffset);
        }
        else
        {
            speedModifier = 1;
            hoodAngle = resualt.y;
            servoOffset = 0;
        }

        
        // MoveHood(hoodAngle);
        
        shooterMotor->Set(shooterPower);
        currentState = State::Shoot;


        currentFlywheelPowerEntry.SetDouble(shooterPower);
        currentFlywheelRPMEntry.SetDouble(shooterPower);
    }
    void ShooterModule::Stop()
    {
        // shooingDistanceTable.SaveToFile(Constants::HOME_DIRECTORY + Constants::Shooter::SHOOTING_DISTANCE_LOOKUP_TABLE_NAME);

        shooterMotor->Set(0);
        currentState = Idle;
    }
    void ShooterModule::PassBall()
    {
        MoveHood(Constants::Shooter::HOOD_MAX_UP_ANGLE);
        shooterMotor->Set(1);
        currentState = Pass;
    }

    void ShooterModule::MoveHood(float angle)
    {
        angle = std::clamp<float>(angle, Constants::Shooter::HOOD_MIN_DOWN_ANGLE, Constants::Shooter::HOOD_MAX_UP_ANGLE);
        
        double hoodPos = Constants::Shooter::HOOD_TABLE.Get(angle);
        
        hoodServo.Set(hoodPos);
    }


    ShooterModule::State ShooterModule::GetState()
    {
        return currentState;
    }

}