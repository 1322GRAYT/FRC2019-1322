/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */



public class RobotMap {

	/***********************************************
   * Drive System
   */
  
  public static final int[] EncoderDriveAddresses = {0, 2, 4, 6};
  public static final int[] FollowerDriveAddresses = {1, 3, 5, 7};

  /***********************************************
   * Lift
   */
  public static final int[] LiftMotorAddresses = {8, 9};

	/***********************************************
   * Air Things
   */

  public static final int[] ClawSolenoids = {0, 1};
  public static final int[] EjectSolenoids = {2, 3};


  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
