
#include <core/LimelightHelpers.h>
#include <frc/Timer.h>
#include "Modules/TurretModule.h"
#include "Constants.h"


#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
    



Turret_Tracking::Turret_Tracking()
{
	turret_motor = new ctre::phoenix6::hardware::TalonFX(Constants::Turret::turretID, Constants::CANIVOUR_NAME);

  	PIDTimer = new Core::Timer();
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

}

Turret_Tracking::~Turret_Tracking()
{
    delete(PIDTimer);
    delete(PIDController);
};

void Turret_Tracking::Update()
{      
	std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	tx = LimelightHelpers::getTX("limelight");  // Horizontal offset from crosshair to target in degrees
	
	limelight_Error = tx * angleoffset;
	angleoffset = 1; // Calibrate the motor encoder value per degree
	motorangle = currentpos / angleoffset; // Output
  	
	
	
	std::cout << "tx: " << tx << '\n';
	std::cout << "tx2 : " << table.get()->GetNumber("tx", 0.0) << '\n';

	hasTarget = LimelightHelpers::getTV("limelight"); // Do you have a valid target?
	PIDTimer->Update();

	
	currentpos = turret_motor->GetPosition().GetValue().value(); // Motors Encoder Value

	std::cout << "encoder pose: " << turret_motor->GetPosition().GetValue().value() << '\n';
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

}

//find april is for looking for the april tag if it cant find it
void Turret_Tracking::Find_april()
{
    if (hasTarget == false && motorangle < 170)
	{
		turret_motor->Set(PIDController->Calculate(currentpos,minRotation * angleoffset,PIDTimer->GetDeltaTime()));
    }
    if (hasTarget == false && motorangle > -170)
	{
		turret_motor->Set(PIDController->Calculate(currentpos,maxRotation * angleoffset,PIDTimer->GetDeltaTime()));
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
	std::cout << "power :" << -PIDController->Calculate(0, limelight_Error, PIDTimer->GetDeltaTime()) << "\n";
	
	
    
	if (hasTarget == true)
	{
        turret_motor->Set(PIDController->Calculate(0, limelight_Error, PIDTimer->GetDeltaTime()));
    }
    else 
	{
      	// Find_april();
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
