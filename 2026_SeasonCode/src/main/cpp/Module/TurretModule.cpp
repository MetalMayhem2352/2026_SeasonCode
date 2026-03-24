
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
	

  	maxRotation = 90;
  	minRotation = -90;

	this->swerveDrive = swerveDrive;
}

Turret_Tracking::~Turret_Tracking()
{
    delete(pidTimer);
    delete(PIDController);
};

void Turret_Tracking::Update()
{      
	
}

//find april is for looking for the april tag if it cant find it
void Turret_Tracking::Find_april()
{
    
}
void Turret_Tracking::turretIdle(){
	turret_motor->Set(0);
}
// tracks april tag for turret tracking
void Turret_Tracking::Track()
{
}
double Turret_Tracking::limelight_Distance()
{
}
