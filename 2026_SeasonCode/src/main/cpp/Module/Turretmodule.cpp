
#include <core/LimelightHelpers.h>
#include <frc/Timer.h>
#include "Modules/TurretModule.h"
#include "Constants.h"


    
 

  
  //calibrate function is to determine what way the turret is rotating
  Turret_Tracking::Turret_Tracking(){

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

  currentpos = 0; // Motors Encoder Value
  double angleoffset = 0; // Calibrate the motor encoder value per degree
  motorangle = currentpos / angleoffset; // Output

  }
  Turret_Tracking::~Turret_Tracking(){
    delete(PIDTimer);
    delete(PIDController);
  };

  void Turret_Tracking::Update(){
      tx = LimelightHelpers::getTX("");  // Horizontal offset from crosshair to target in degrees
      hasTarget = LimelightHelpers::getTV(""); // Do you have a valid target?
      PIDTimer->Update();
  }

  int Turret_Tracking::Calibrate(){
    timer.Start();
    turret_motor.Set(0.1);
    if (timer.HasElapsed(units::time::second_t(1.0))){
      if (motorangle > 0){
        forward = 1;
        backward = -1;
      }
      else if (motorangle < 0) {
        forward = -1;
        backward = 1;
      }
      timer.Reset();
    }
    
  }
  //find april is for looking for the april tag if it cant find it
  int Turret_Tracking::Find_april(){
    if(hasTarget == false && motorangle > maxRotation){
      turret_motor.Set(backward);
    }
    else if(hasTarget == false && motorangle < minRotation){
      turret_motor.Set(forward);
    }
  }
  // tracks april tag for turret tracking
  int Turret_Tracking::Track(){
    if (hasTarget == true){
        turret_motor.Set(PIDController->Calculate(currentpos,tx,PIDTimer->GetDeltaTime()));
    }
  }

