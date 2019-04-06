/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

/**
 * Class: K_Drive - Contains Control Calibrations for the Drive control system.
 */
public class K_Drive {

    /**************************************************/
    /*  Drive System Feed-Forward Term                */
 	/**************************************************/	 	

	/** KeDRV_dPct_VelFeedFwdTerm: Drive System Velocity Control
     * Feed Forward Term for Motion Magic Control in units of
     * 1023 * duty-cycle / encoder-counter-per-100ms.
     */
    public static final double KeDRV_dPct_VelFeedFwdTerm = 0.073;


    /**************************************************/
    /*  Drive System PID Gains                       */
 	/**************************************************/	 	

	/** KeDRV_k_VelPropGx: Drive System Velocity Control
     * Proportional Gain.
     */
    public static final double KeDRV_k_VelPropGx = 0.2;
     
	/** KeDRV_k_VelIntglGx: Drive System Velocity Control
     * Integral Gain.
     */
    public static final double KeDRV_k_VelIntglGx = 0.00007;

	/** KeDRV_k_VelDerivGx: Drive System Velocity Control
     * Derivative Gain.
     */
    public static final double KeDRV_k_VelDerivGx = 0.0013;

    /*******************************************************/
    /*  Drive System Closed-Loop Target Longitudal Control */
 	/*******************************************************/	 	

	/** KeDRV_r_CL_NormPwrLong: Drive System Normalized Power Command
     * for Driving the Robot forward while tracking the vision target
     * via Closed-Loop Control.
     */
    public static final double KeDRV_r_CL_NormPwrLong = 0.5;

	/** KeDRV_r_CL_ScalarRotToLat: Drive System Normalized Power Command
     * Scalar to Convert Closed-Loop Rotational Power Command to Closed-Loop
     * Lateral Power Command when the robot is driving foward while target
     * tracking.
     */
    public static final double KeDRV_r_CL_ScalarRotToLat = 0.5;


   
    /**************************************************/
    /*  Drive System Motion Speed Profile Settings    */
    /**************************************************/	 	

	/** KeDRV_n_MM_CruiseVel: Drive System Motion Magic
     * Speed Profiling Cruise Velocity in units of encoder
     * counts per 100ms.
     */
    public static final int KeDRV_n_MM_CruiseVel = 9000;
     
	/** KeDRV_a_MM_MaxAccel: Drive System Motion Magic
     * Speed Profiling Maximum Acceleration in units of
     * encoder counts per 100ms per sec.
     */
    public static final int KeDRV_a_MM_MaxAccel = 13000;

   
    /**************************************************/
    /*  Drive System Driver Control Deadbands         */
    /**************************************************/	 	

	/** KeDRV_r_DB_InpForeAft: Drive System Controller Driver Controls
      * Input Power Deadband when detecting a direction request for Longitudinal
      * robot movement. (Normalized Power Command) 
     */
    public static double KeDRV_r_DB_InpLong = 0.2;
     

	/** KeDRV_r_DB_InpLat: Drive System Controller Driver Controls
      * Input Power Deadband when detecting a direction request for Lateral
      * robot movement. (Normalized Power Command)
     */
    public static double KeDRV_r_DB_InpLat = 0.2;


	/** KeDRV_r_DB_InpRot: Drive System Controller Driver Controls
      * Input Power Deadband when detecting a direction request for Rotational
      * robot movement. (Normalized Power Command)
      */
    public static double KeDRV_r_DB_InpRot = 0.2;

	/** KeDRV_Deg_DB_Dirctn: Drive System Controller Driver Controls
      * Input Direction Deadband when detecting a direction request for longitudinal
      * or latitudinal robot movement. (Degrees) 
     */
    public static double KeDRV_Deg_DB_Dirctn = 0.2;
     


    /**************************************************/
    /*  Drive System Speed Ratio / Conversion Cals    */
    /**************************************************/	 	

    /* KeDRV_l_DistPerRevWhl: Linear Distance Traveled Forward/Rearward
     * per one Wheel Revolution (inches), Pi*D */
     public static final float KeDRV_l_DistPerRevWhl = (float) (Math.PI * 4);  // inches

    /* KeDRV_Cnt_PlsPerRevEncdr: Number of Shaft Encoder Pulses per one
     * rotation of the Shaft the Encoder is mounted on.  (Encoder Teeth * 4) */
     public static final int KeDRV_Cnt_PlsPerRevEncdr = (int) 1000;    // clicks
  
    /* KeDRV_r_EncdrToWhl: The ratio of the number of rotations of
     * the encoder shaft to the number of rotations of the wheel axle shaft,
     * i.e. Gear Ratio: 56/16 * 40/20 = 7.0. 
     * */  
     public static final float KeDRV_r_EncdrToWhl = (float) 7.0;      // ratio
  
    /* KeDRV_n_EncdrSpdMaxLim: The Unloaded Encoder Speed Recorded for the
     * slowest Wheel/Motor during testing which will be used as the Maximum
     * Encoder Speed Limit (Error Protection).  */  
     public static final float KeDRV_n_EncdrSpdMaxLim = (float) 3000.0;  // rev/min

}
