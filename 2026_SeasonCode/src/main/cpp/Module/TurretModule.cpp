
#include <core/LimelightHelpers.h>
#include <frc/Timer.h>
#include "Modules/TurretModule.h"
#include "Constants.h"


#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"

    



Turret_Tracking::Turret_Tracking(CustomSwerveDrive::SwerveDriveModule* swerveDrive)
{
	turret_motor = new ctre::phoenix6::hardware::TalonFX(Constants::Turret::turretID, Constants::CANIVOUR_NAME);

  	pidTimer = new Core::Timer();
  	PIDController = new Core::PIDController(Constants::Turret::TurretPIDConfig);

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

  	maxRotation = 180;
  	minRotation = -180;

	camera_height = 113.0;
	target_height = 128.0;
	camera_angle = 0;

  	currentpos = turret_motor->GetPosition().GetValue().value(); // Motors Encoder Value
  	angleoffset = 1; // Calibrate the motor encoder value per degree
  	limelight_Error = tx * angleoffset;
  	motorangle = currentpos / angleoffset; // Output

	this->swerveDrive = swerveDrive;
}

Turret_Tracking::~Turret_Tracking()
{
    delete(pidTimer);
    delete(PIDController);
};

void Turret_Tracking::Update()
{
	/*
	
	tx = LimelightHelpers::getTX("limelight");  // Horizontal offset from crosshair to target in degrees
	hasTarget = LimelightHelpers::getTV("limelight"); // Do you have a valid target?
	
	limelight_Error = tx * angleoffset;
	angleoffset = 1; // Calibrate the motor encoder value per degree
	motorangle = currentpos / angleoffset; // Output
	
	
	
	currentpos = turret_motor->GetPosition().GetValue().value(); // Motors Encoder Value
	
	angleoffset = 1; // Calibrate the motor encoder value per degree
	
	motorangle = currentpos / angleoffset; // Output
	
	if(limelight_Error > maxRotation)
	{
		desiredEncoderPosition = maxRotation * angleoffset;
	}
	if (desiredEncoderPosition < minRotation * angleoffset)
	{
		desiredEncoderPosition = minRotation * angleoffset;
	}
    */

   	pidTimer->Update();
   
	// NEw Stuff.
	Rotate(0);
}

//find april is for looking for the april tag if it cant find it
void Turret_Tracking::Find_april()
{
    if (hasTarget == false && motorangle < 170)
	{
		turret_motor->Set(PIDController->Calculate(currentpos,minRotation * angleoffset,pidTimer->GetDeltaTime()));
    }
    if (hasTarget == false && motorangle > -170)
	{
		turret_motor->Set(PIDController->Calculate(currentpos,maxRotation * angleoffset,pidTimer->GetDeltaTime()));
    }
}
void Turret_Tracking::turretIdle(){
	turret_motor->Set(0);
}
// tracks april tag for turret tracking
void Turret_Tracking::Track()
{
	Update();

	std::cout << "limelight_Error: " << limelight_Error << "\n";
	std::cout << "limelight_Error: " << limelight_Error << "\n";

	if (std::abs(limelight_Error) > 0.5)
	{
        turret_motor->Set(PIDController->Calculate(0, limelight_Error, pidTimer->GetDeltaTime()));
    }
    else 
	{
		turret_motor->Set(0);
    }
}
double Turret_Tracking::limelight_Distance()
{
	bool hasTarget = LimelightHelpers::getTV("limelight"); // 1 if target detected, 0 if not
	double ty = LimelightHelpers::getTY("limelight");      // Vertical offset in degrees

    if (hasTarget) {
        // Convert angles to radians for trig functions
        double angleToTargetRad = (camera_angle + ty) * M_PI / 180.0;

        // Distance formula: (targetHeight - cameraHeight) / tan(angle)
        distance = (target_height - camera_height) / std::tan(angleToTargetRad);

        std::cout << "Target detected!\n";
        std::cout << "Vertical angle (ty): " << ty << " degrees\n";
        std::cout << "Distance: " << distance << " meters\n";
    } 
	else {
        distance = 0.0;
        std::cout << "distance : " << distance << "\n";
    }

		//TA = LimelightHelpers::getTA("limelight");
		//std::cout << "Target_Area : " << TA << "\n";

		//scale = 11672.17;
		//distance = (scale * std::pow(TA, 1.9));
		//std::cout << "distance : " << distance << "\n";
}

bool Turret_Tracking::CanShoot()
{
	return tx < 2;
}

void Turret_Tracking::Rotate(double targetPosition)
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
	
	std::cout << "position: " << position << "\n";
	std::cout << "error: " << error << "\n";
	std::cout << "power: " << power << "\n";

	turret_motor->Set(power);
}

double Turret_Tracking::GetTurretPosition()
{
	return turret_motor->GetPosition().GetValueAsDouble() * Constants::Turret::MOTOR_TICKS_PER_DEGREE;
}

void Turret_Tracking::Pass()
{

}

bool Turret_Tracking::CanPass()
{
	
}