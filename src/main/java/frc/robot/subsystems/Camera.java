/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Robot;


/**
 * Add your docs here.
 */
public class Camera extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Network Table for Communications
  NetworkTableInstance NetTbl;

  /* Active CameraServer Camera */
  private int Active_Camera;


  public void Camera() {
    Active_Camera = 0; // Default to camera 0
  }


  /**
   * Method: SlctCAM_DrvrCam - Switches between the Visiual Driver-View cameras.
   */ 
  public void SlctCAM_DrvrCam() {
    if(this.Active_Camera == 0) { // Camera is 0, set to 1
      this.NetTbl.getTable("").getEntry("CameraSelection").setString(Robot.camera1.getName());
      this.Active_Camera = 1;
    } else { // Camera is 1, set to 0
      this.NetTbl.getTable("").getEntry("CameraSelection").setString(Robot.camera0.getName());
      this.Active_Camera = 0;
    }
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
