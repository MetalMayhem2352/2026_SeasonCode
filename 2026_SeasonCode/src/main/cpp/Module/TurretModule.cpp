
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

	this->swerveDrive = swerveDrive;
}

Turret_Tracking::~Turret_Tracking()
{
    delete(pidTimer);
    delete(PIDController);
};

void Turret_Tracking::Update()
{      
	std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	tx = LimelightHelpers::getTX("limelight");  // Horizontal offset from crosshair to target in degrees
	
	limelight_Error = tx * angleoffset;
	angleoffset = 5.556; // Calibrate the motor encoder value per degree
	motorangle = currentpos / angleoffset; // Output
  	
	

	hasTarget = LimelightHelpers::getTV("limelight"); // Do you have a valid target?
	pidTimer->Update();

	lockedTargetHeading = swerveDrive->GetYaw() + tx;
	targetAngle = error;
	targetticks = targetAngle * angleoffset * 5;

	if (motorangle < -60)
	{
		targetticks = motorangle * angleoffset;
	}
	if (motorangle < 60)
	{
		targetticks = motorangle * angleoffset;
	}

	error = lockedTargetHeading - swerveDrive->GetYaw();

	
	currentpos = turret_motor->GetPosition().GetValue().value(); // Motors Encoder Value

	std::cout << "encoder pose: " << turret_motor->GetPosition().GetValue().value() << '\n';

	

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

	std::cout << "error :" << limelight_Error << "\n";
	std::cout << "pos :" << currentpos << "\n";
	std::cout << "power :" << -PIDController->Calculate(0, limelight_Error, pidTimer->GetDeltaTime()) << "\n";
	
    turret_motor->Set(PIDController->Calculate(currentpos, targetticks, pidTimer->GetDeltaTime()));
    
}
double Turret_Tracking::limelight_Distance()
{
	

	bool hasTarget = LimelightHelpers::getTV("limelight"); // 1 if target detected, 0 if not
	double ty = LimelightHelpers::getTY("limelight");      // Vertical offset in degrees

    if (hasTarget) {
        
    } 
}