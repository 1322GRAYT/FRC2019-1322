
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.calibrations;



/**
 * Class: K_PID - Contains the Sensor Calibrations for the
 * the Target Tracking PID Controller.
 */
public class K_PID {

	
  /** KePID_t_LoopRt: Execution Loop Rate Period for the Target
    * System PID Controller (sec). */
	  public static final float KePID_t_LoopRt = (float)0.020; // sec 
  
	    
  /*****************************************************************/
  /* Drive Target Tracking PID Control Calibrations                */
  /*****************************************************************/	    
  
  /** KnPID_Deg_FdFwdErrAxis: Angle Error Axis for the Robot
    * Rotation Control FeedFoward Correction Term for PI
    * Control (Degrees). */
 public static final int KnPID_Deg_FdFwdErrAxis[] = new int[] 
     {
	     (int)0,
       (int)1,
       (int)2,
       (int)3,
   	   (int)4,
       (int)5,
       (int)8,
       (int)10,
       (int)15,
       (int)20
	  };	  
	  
 
 /** KtPID_Pct_FdFwdCorr: Target Percent Power Axis for the Target
  * Power Launch Profile Shaping Tables. (Percent Power). */
 public static final float KtPID_Pct_FdFwdCorr[] = new float[] 
     {
	    (float)0.0,
      (float)0.0,
      (float)2.0,
      (float)5.0,
   	  (float)10.0,
      (float)15.0,
      (float)20.0,
      (float)25.0,
      (float)30.0,
      (float)30.0
	  };	  
  
  
  /** KePID_Deg_PosErrDB: Drive System Rotate Position Error DeadBand (degree). */
  public static final float KePID_Deg_PosErrDB = (float) 1.0;

  /** KePID_Deg_IntglErrDsblMin: Drive System Rotate Absolute Error Threshold
   * outside of which the Integral Correction will be disabled.  Do not start
   * applying Integral Correction until within this band. (degree) */
  public static final float KePID_Deg_IntglErrDsblMin = (float) 30.0;
  
  /** KePID_K_PropGx: Drive System Rotate PID Controls Proportional Gain. */
  public static final float KePID_K_PropGx = (float) 2.0;
  
  /** KePID_K_IntglGx: Drive System Rotate PID Controls Integral Gain. */
  public static final float KePID_K_IntglGx = (float) 1.0;
 
  /** KePID_Pct_PropCorrMax: Drive System Rotate PID Controls Proportional
   * Correction Max Limit. (percent) */
  public static final float KePID_Pct_PropCorrMax = (float) 45.0;
  
  /** KePID_Pct_PropCorrMax: Drive System Rotate PID Controls Integral
   * Correction Max Limit. (percent) */
  public static final float KePID_Pct_IntglCorrMax = (float) 40.0;
  
  /** KePID_t_PstnTgtSyncMetThrsh: Amount of time that the Position Error must
   * be held within the Error DeadBand in order for the Drive System Rotate
   * PID Controls to consider the Position Target conditions met. (seconds) */
  public static final float KePID_t_PstnTgtSyncMetThrsh = (float) 0.200;
   
}
