#include "Modules/NewTurretModule.h" 

#include <iostream>

namespace Modules
{
	NewTurretModule::NewTurretModule()
	{
		turret_motor = new ctre::phoenix6::hardware::TalonFX(Constants::Turret::turretID, Constants::CANIVOUR_NAME);
		
		pidTimer = new Core::Timer();
		PIDController = new Core::PIDController(Constants::Turret::TurretPIDConfig2);

		
        nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
        
        inst.SetServerTeam(2352);
        inst.StartClient4("MyConsoleClient");
        
        std::shared_ptr<nt::NetworkTable> networkTable = inst.GetTable("SmartDashboard");


		kP_Entry = networkTable->GetEntry("turret_kP");
        kPDeadzone_Entry = networkTable->GetEntry("turret_kPDeadzone");
        kI_Entry = networkTable->GetEntry("turret_kI");
        kIActiveZone_Entry = networkTable->GetEntry("turret_kIActiveZone");
        kD_Entry = networkTable->GetEntry("turret_kD");

        minAngleEntry = networkTable->GetEntry("turretMinAngle");
        maxAngleEntry = networkTable->GetEntry("turretMaxAngle");
        
		targetAngleEntry = networkTable->GetEntry("taregtTurretAngle");
        
		currentPositionEntry = networkTable->GetEntry("currentTurrentPosition");

        currentVelocityModifierEntry = networkTable->GetEntry("currentVelecoityModifier");
        predictedVelocityModifierEntry = networkTable->GetEntry("predictedVelecoityModifier");

        // kP_Entry.SetDouble(0);
        // kPDeadzone_Entry.SetDouble(0);
        // kI_Entry.SetDouble(1);
        // kIActiveZone_Entry.SetDouble(1);
        // kD_Entry.SetDouble(1);

        minAngleEntry.SetDouble(Constants::Turret::MIN_ROTATION);
        maxAngleEntry.SetDouble(Constants::Turret::MAX_ROTATION);

        targetAngleEntry.SetDouble(0);

        currentPositionEntry.SetDouble(0);

        currentVelocityModifierEntry.SetDouble(0);
        predictedVelocityModifierEntry.SetDouble(0);
	}

	NewTurretModule::~NewTurretModule()
	{
		delete(pidTimer);
		delete(PIDController);
	};

	void NewTurretModule::Update(frc::Pose2d robotPosition, frc::ChassisSpeeds velocity, double xInput, double zInput, double rotationInput)
	{
        frc::Pose2d targetPosition;
        switch (currentState)
        {
            case Shoot:
                targetPosition = Constants::goalPosition;
                break;
            case PassLeft:
                break;
            case PassRight:
                targetPosition = Constants::zeroPosition;
                break;
        };

        double target = GetTargetPosition(targetPosition, robotPosition, velocity, xInput, zInput, rotationInput) - 180;
	
		Rotate(target);
    }
	void NewTurretModule::UpdateState(State newState)
	{
        currentState = newState;
	}

	double NewTurretModule::GetTurretPosition()
	{
		return turret_motor->GetPosition().GetValueAsDouble() * Constants::Turret::MOTOR_TICKS_PER_DEGREE;
	}
	bool NewTurretModule::CanShoot()
	{
        return true;
	}

    
	void NewTurretModule::Rotate(double targetPosition)
	{
		// Normallizing pos between -180 and 180
		while (targetPosition < -180)
		{
			targetPosition += 360;
		}
		while (targetPosition > 180)
		{
			targetPosition -= 360;
		}

		if (targetPosition < Constants::Turret::MIN_ROTATION + Constants::Turret::TOLERANCE)
		{
			targetPosition = Constants::Turret::MIN_ROTATION + Constants::Turret::TOLERANCE;
		}
		else if (targetPosition > Constants::Turret::MAX_ROTATION - Constants::Turret::TOLERANCE)
		{
			targetPosition = Constants::Turret::MAX_ROTATION - Constants::Turret::TOLERANCE;
		}

		double position = GetTurretPosition();
		// double error = targetPosition - position;
		double power = PIDController->Calculate(position, targetPosition, pidTimer->GetDeltaTime());
		
		turret_motor->Set(power);
	}
	
    double NewTurretModule::GetTargetPosition(frc::Pose2d targetPosition, frc::Pose2d robotPosition, frc::ChassisSpeeds velocity, double xInput, double zInput, double rotationInput)
    {
        double zDistance = targetPosition.X().value() - robotPosition.X().value();
        double xDistance = -targetPosition.Y().value() - (-robotPosition.Y().value());
	
        double targetAngle = std::atan2(zDistance, xDistance) * Constants::RADIANS_TO_DEGREES;
		

        robotPosition.Rotation().Degrees();
        return targetAngle + robotPosition.Rotation().Degrees().value();
    }

}

