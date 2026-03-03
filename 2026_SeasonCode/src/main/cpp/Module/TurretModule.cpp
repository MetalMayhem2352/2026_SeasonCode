
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

  	
  	angleoffset = 0.18; // Calibrate the motor encoder value per degree
  	
  	

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
	tx;
	hasTarget;
	currentpos = turret_motor.GetPosition().GetValue().value(); // Motors Encoder Value 
	motorangle = currentpos / angleoffset; // Output
	desiredEncoderPosition = currentpos + (tx * angleoffset);
	
	PIDTimer->Update();
	if(desiredEncoderPosition > maxRotation * angleoffset)
	{
		desiredEncoderPosition = maxRotation * angleoffset;
	}
	if (desiredEncoderPosition < minRotation * angleoffset)
	{
		desiredEncoderPosition = minRotation * angleoffset;
	}
}

//find april is for looking for the april tag if it cant find it
int Turret_Tracking::Find_april()
{
	sweepingRight = true;
    if (!hasTarget)
{
    if (motorangle >= maxRotation)
        sweepingRight = false;

    if (motorangle <= minRotation)
        sweepingRight = true;

    double sweepTarget = sweepingRight ? 
        maxRotation * angleoffset :
        minRotation * angleoffset;

    turret_motor.Set(
        PIDController->Calculate(
            currentpos,
            sweepTarget,
            PIDTimer->GetDeltaTime()
        )
    );
}
}

// tracks april tag for turret tracking
int Turret_Tracking::Track()
{
	Turret_Tracking::Update();
    if (hasTarget == true){
        turret_motor.Set(PIDController->Calculate(currentpos,desiredEncoderPosition,PIDTimer->GetDeltaTime()));
    }
    else {
      	//Find_april();
    }
}
