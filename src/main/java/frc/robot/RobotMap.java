/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import javax.lang.model.util.ElementScanner6;

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
  
  // CAN Talons
  public static final int[] EncoderDriveAddresses = {0, 2, 4, 6};
  public static final int[] FollowerDriveAddresses = {1, 3, 5, 7};

  /***********************************************
   * Lift
   */
  
  // CAN Talons
  public static final int LiftMotorAddress = 8;
  public static final int BallIntakeAddress = 9;


  /***********************************************
   * Robot Lift
   */

  public static final int[] RobotLiftAddresses = {10,11};
  public static final int RobotLiftExtendAddress = 12;
  //Relays
  public static final int RobotScissorLock = 2;  
  public static final int PracticeBotCompressor = 1; // Practice Bot only!  






	/***********************************************
   * Air Things Competition Bot
   */

  public static final int[] ClawSolenoids = {2, 5};
  public static final int[] EjectSolenoids = {3, 4};
  public static final int[] RobotLift = {1, 6}; // This doesn't exist on the test bot

  	/***********************************************
   * Sensors
   */

  public static final int ClawSensor = 1;
  public static final int FloorSensor = 2;
  public static final int DiskSensor = 4;

  /***********************************************
   * Arm Lift
   */

  // CAN Talons
  public static final int LIFT_1    = 8;
	public static final int LIFT_2    = 9;
	public static final int CLAW_L    = 10;
  public static final int CLAW_R    = 11;
  
  // Digital Input
  public static final int BLOCK_DETECTOR = 0;
	public static final int LOW_LIFT = 2;
	public static final int MID_LIFT = 3;
	public static final int HIGH_LIFT = 4;


  
}