
#include <core/LimelightHelpers.h>
#include <frc/Timer.h>
#include "Modules/TurretModule.h"
#include "Constants.h"


#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
    

namespace Modules
{
	TurretModule::TurretModule()
	{
		turret_motor = new ctre::phoenix6::hardware::TalonFX(Constants::Turret::turretID, Constants::CANIVOUR_NAME);
		
		pidTimer = new Core::Timer();
		PIDController = new Core::PIDController(Constants::Turret::TurretPIDConfig2);

        pigeon = new ctre::phoenix6::hardware::Pigeon2(Constants::pigeonID, Constants::CANIVOUR_NAME);

		LimelightHelpers::setPipelineIndex("limelight",0);


		LimelightHelpers::SetupPortForwardingUSB(0);

		std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
		tx = LimelightHelpers::getTX("limelight");  // Horizontal offset from crosshair to target in degrees
		double ty = LimelightHelpers::getTY("limelight");  // Vertical offset from crosshair to target in degrees
		double ta = LimelightHelpers::getTA("limelight");  // Target area (0% to 100% of image)
		hasTarget = LimelightHelpers::getTV("limelight"); // Do you have a valid target?

		double txnc = LimelightHelpers::getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
		double tync = LimelightHelpers::getTYNC("limelight");  // Vertical offset from principal pixel/point to target in degrees

		bool looking = false;
		//robotYaw = pigeon->GetYaw().GetValue();
		

		maxRotation = 90;
		minRotation = -90;

		camera_height = 113.0;
		target_height = 128.0;
		camera_angle = 0;

		currentpos = turret_motor->GetPosition().GetValue().value(); // Motors Encoder Value
		angleoffset = 5.556; // Calibrate the motor encoder value per degree
		limelight_Error = tx * angleoffset;
		motorangle = currentpos / angleoffset; // Output
	}

	TurretModule::~TurretModule()
	{
		delete(pidTimer);
		delete(PIDController);

        delete(pigeon);
	};

	void TurretModule::Update()
	{      
		std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
		tx = LimelightHelpers::getTX("limelight-main");  // Horizontal offset from crosshair to target in degrees
		
		limelight_Error = tx * angleoffset;
		angleoffset = 0.05; // Calibrate the motor encoder value per degree
		motorangle = currentpos / angleoffset; // Output
  	
	

		hasTarget = LimelightHelpers::getTV("limelight"); // Do you have a valid target?
		pidTimer->Update();
		if (hasTarget)
		{
			lockedTargetHeading = GetYaw() + tx;
		}
		error = lockedTargetHeading -  GetYaw();
		while (error > 180) error -= 360;
		while (error < -180) error += 360;

		targetAngle = error;
		targetticks = targetAngle * angleoffset;

		/*if (targetticks < -5.4)
		{
			targetticks = -5.4;
		}
		if (targetticks > 4.5)
		{
			targetticks = 4.5;
		}*/
	
	

	
		currentpos = turret_motor->GetPosition().GetValue().value(); // Motors Encoder Value

		std::cout << "encoder pose: " << turret_motor->GetPosition().GetValue().value() << '\n';
	}
	void TurretModule::turretIdle(){
		turret_motor->Set(0);
	}
	// tracks april tag for turret tracking
	void TurretModule::Track()
	{
		Update();

		std::cout << "yaw :" << GetYaw() << "\n";
		std::cout << "targetAngle :" << targetAngle << "\n";
		std::cout << "tx :" << tx << "\n";
		
		// turret_motor->Set(PIDController->Calculate(currentpos, targetticks, pidTimer->GetDeltaTime()));
		
		Rotate(targetAngle);
	}
	double TurretModule::limelight_Distance()
	{
		

		bool hasTarget = LimelightHelpers::getTV("limelight"); // 1 if target detected, 0 if not
		double ty = LimelightHelpers::getTY("limelight");      // Vertical offset in degrees

		if (hasTarget) {
			
		} 

		return 0;
	}

	/// @brief Degrees
    /// @return The YAW of the giro in degrees.
    double TurretModule::GetYaw()
    {
        double yaw = pigeon->GetYaw().GetValue().value();

        yaw = 360 - yaw;

        while (yaw < 0)
        {
            yaw += 360;
        }
        return std::fmod(yaw, 360.0);
    }

	
	void TurretModule::Rotate(double targetPosition)
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
		double error = targetPosition - position;
		double power = PIDController->Calculate(position, targetPosition, pidTimer->GetDeltaTime());
		
		turret_motor->Set(power);
	}

	double TurretModule::GetTurretPosition()
	{
		return turret_motor->GetPosition().GetValueAsDouble() * Constants::Turret::MOTOR_TICKS_PER_DEGREE;
	}
}

