
#include <core/LimelightHelpers.h>
#include <frc/Timer.h>
#include "Modules/TurretModule.h"
#include "Constants.h"


    
 

  
Turret_Tracking::Turret_Tracking()
{
  	PIDTimer = new Core::Timer();
  	PIDController = new Core::PIDController(Constants::Turret::TurretPIDConfig);

  	LimelightHelpers::setPipelineIndex("",0);

  	tx = LimelightHelpers::getTX("");  // Horizontal offset from crosshair to target in degrees
  	double ty = LimelightHelpers::getTY("");  // Vertical offset from crosshair to target in degrees
  	double ta = LimelightHelpers::getTA("");  // Target area (0% to 100% of image)
  	hasTarget = LimelightHelpers::getTV(""); // Do you have a valid target?

  	double txnc = LimelightHelpers::getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
  	double tync = LimelightHelpers::getTYNC("");  // Vertical offset from principal pixel/point to target in degrees

  	bool looking = false;

  	maxRotation = 180;
  	minRotation = -180;

  	currentpos = turret_motor.GetPosition().GetValue().value(); // Motors Encoder Value
  	angleoffset = 0; // Calibrate the motor encoder value per degree
  	error = tx * angleoffset;
  	motorangle = currentpos / angleoffset; // Output

}

Turret_Tracking::~Turret_Tracking()
{
    delete(PIDTimer);
    delete(PIDController);
};

void Turret_Tracking::Update()
{      
	tx = LimelightHelpers::getTX("");  // Horizontal offset from crosshair to target in degrees
	hasTarget = LimelightHelpers::getTV(""); // Do you have a valid target?
	PIDTimer->Update();
	if(error > maxRotation)
	{
		error = maxRotation;
	}
	if (error < minRotation)
	{
		error = minRotation;
	}
}

//find april is for looking for the april tag if it cant find it
int Turret_Tracking::Find_april()
{
    if (hasTarget == false && motorangle > 170)
	{
      	turret_motor.Set(PIDController->Calculate(currentpos,minRotation * angleoffset,PIDTimer->GetDeltaTime()));
    }
    if (hasTarget == false && motorangle < -170)
	{
      	turret_motor.Set(PIDController->Calculate(currentpos,maxRotation * angleoffset,PIDTimer->GetDeltaTime()));
    }
}

// tracks april tag for turret tracking
int Turret_Tracking::Track()
{
    if (hasTarget == true){
        turret_motor.Set(PIDController->Calculate(currentpos,error,PIDTimer->GetDeltaTime()));
    }
    else {
      	Find_april();
    }
}
