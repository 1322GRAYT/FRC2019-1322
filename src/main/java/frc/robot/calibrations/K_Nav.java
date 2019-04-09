/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

/**
 * Class: K_Nav - Contains Control Calibrations for the Navigation control system.
 */
public class K_Nav {

  /************************************************************/
  /*  Navigation System Closed-Loop Target Longitudal Control */
 	/************************************************************/	 	

	/** KeNAV_r_CL_NormPwrLong: Drive System Normalized Power Command
    * for Driving the Robot forward while tracking the vision target
    * via Closed-Loop Control.
    */
    public static final double KeNAV_r_CL_NormPwrLong = 0.5;

	/** KeNAV_r_CL_ScalarRotToLat: Drive System Normalized Power Command
    * Scalar to Convert Closed-Loop Rotational Power Command to Closed-Loop
    * Lateral Power Command when the robot is driving foward while target
    * tracking.
    */
    public static final double KeNAV_r_CL_ScalarRotToLat = 0.5;


	/** KeNAV_Deg_DB_Dirctn: Drive System Controller Driver Controls
    * Input Direction Deadband when detecting a direction request for longitudinal
    * or latitudinal robot movement. (Degrees) 
    */
    public static double KeNAV_Deg_DB_Dirctn = 0.2;



  /**************************************************/
  /*  Drive System Speed Ratio / Conversion Cals    */
  /**************************************************/	 	

  /**  KeNAV_l_DistPerRevWhl: Linear Distance Traveled Forward/Rearward
    * per one Wheel Revolution (inches), Pi*D
    */
    public static final float KeNAV_l_DistPerRevWhl = (float) (Math.PI * 4);  // inches

  /** KeNAV_Cnt_PlsPerRevEncdr: Number of Shaft Encoder Pulses per one
    * rotation of the Shaft the Encoder is mounted on.  (Encoder Teeth * 4)
    */
    public static final int KeNAV_Cnt_PlsPerRevEncdr = (int) 1024 * 4; // 1024 ticks at 4x encoder (4x Encoder counts - Up and Down)
  
  /** KeNAV_r_EncdrToWhl: The ratio of the number of rotations of
    * the encoder shaft to the number of rotations of the wheel axle shaft,
    * i.e. Gear Ratio: 56/16 * 40/20 = 7.0. 
    */  
    public static final float KeNAV_r_EncdrToWhl = (float) 7.0;      // ratio
  
  /** KeNAV_n_EncdrSpdMaxLim: The Unloaded Encoder Speed Recorded for the
    * slowest Wheel/Motor during testing which will be used as the Maximum
    * Encoder Speed Limit (Error Protection).
    */  
    public static final float KeNAV_n_EncdrSpdMaxLim = (float) 3000.0;  // rev/min

}
