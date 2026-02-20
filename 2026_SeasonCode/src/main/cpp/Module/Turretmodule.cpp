
#include <LimelightHelpers.h>
#include <ctre/Phoenix.h>
#include <frc/Timer.h>

public class Turret_Tracking {
    
  frc::Timer timer;

  double forward = 0
  double backwards = 0

  WPI_TalonFX turret_motor{1};

  limelight = LimelightHelpers::SetupPortForwardingUSB(0);
  LimelightHelpers::setPipelineIndex("",0)

  double tx = LimelightHelpers::getTX("");  // Horizontal offset from crosshair to target in degrees
  double ty = LimelightHelpers::getTY("");  // Vertical offset from crosshair to target in degrees
  double ta = LimelightHelpers::getTA("");  // Target area (0% to 100% of image)
  bool hasTarget = LimelightHelpers::getTV(""); // Do you have a valid target?

  double txnc = LimelightHelpers::getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
  double tync = LimelightHelpers::getTYNC("");  // Vertical offset from principal pixel/point to target in degrees

  bool looking = false;

  double maxrotation = 180;
  double minrotation = -180;

  double currentpos = 0; // Motors Encoder Value
  double angleoffset = 256; // Calibrate the motor encoder value per degree
  double motorangle = currentpos / angleoffset; // Output

  //calibrate function is to determine what way the turret is rotating
  public int Calibrate(){
    timer.start();
    turret_motor.set(ControlMode::PercentOutput, 0,1)
    if (timer.HasElapsed(1.0)){
      if (motorangle > 0){
        forward = 1
        backward = -1
      }
      else if (motorangle < 0) {
        forward = -1
        backward = 1
      }
      timer.reset();
    }
    
  }
  //find april is for looking for the april tag if it cant find it
  public int find_april(){
    if(hasTarget == false && motorangle > maxrotation){
      turret_motor.set(ControlMode::PercentOutput, backward)
    }
    else if(hasTarget == false && motorangle < minrotation){
      turret_motor.set(ControlMode::PercentOutput, forward)
    }
    
  }

  // tracks april tag for turret tracking
  int track(){
    if (hasTarget == true){
      //PID used for tracking dont have access to ATM
      if(tx > 0){
        turret_motor.set(ControlMode::PercentOutput, forward)
      }
      if(tx < 0){
        turret_motor.set(ControlMode::PercentOutput, backward)
      }
    }
  }
  
  //on start
  if (){
    calibrate();
  }

}