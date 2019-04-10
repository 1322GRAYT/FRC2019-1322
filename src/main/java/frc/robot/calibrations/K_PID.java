
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
  
  /** KnPID_Deg_VSN_FdFwdErrAxis: Angle Error Axis for the Robot
    * Rotation Control FeedFoward Correction Term for PI
    * Control of Vision Tracking (Degrees). */
 public static final float KnPID_Deg_VSN_FdFwdErrAxis[] = new float[] 
     {
	     (float)0.0,
       (float)0.25,
       (float)0.5,
       (float)1.0,
   	   (float)2.0,
       (float)5.0,
       (float)7.5,
       (float)10.0,
       (float)15.0,
       (float)20.0
	  };	  
	  
 
 /** KtPID_Pct_VSN_FdFwdCorr: Robot Rotation Control FeedFoward
  * Correction Term for PI Control of Vision Tracking (Degrees). */
 public static final float KtPID_Pct_VSN_FdFwdCorr[] = new float[] 
     {
	    (float)0.0,
      (float)5.0,
      (float)10.0,
      (float)30.0,
   	  (float)40.0,
      (float)50.0,
      (float)55.0,
      (float)60.0,
      (float)65.0,
      (float)75.0
	  };	  
  
  
  /** KePID_Deg_VSN_PosErrDB: Drive System Rotate Position Error DeadBand (degree). */
  public static final float KePID_Deg_VSN_PosErrDB = (float) 2.0;

  /** KePID_Deg_VSN_IntglErrDsblMin: Drive System Rotate Absolute Error Threshold
   * outside of which the Integral Correction will be disabled.  Do not start
   * applying Integral Correction until within this band. (degree) */
  public static final float KePID_Deg_VSN_IntglErrDsblMin = (float) 5.0;
  
  /** KePID_K_VSN_PropGx: Drive System Rotate PID Controls Proportional Gain. */
  public static final float KePID_K_VSN_PropGx = (float) 3.0;
  
  /** KePID_K_VSN_IntglGx: Drive System Rotate PID Controls Integral Gain. */
  public static final float KePID_K_VSN_IntglGx = (float) 0.05;
 
  /** KePID_Pct_VSN_PropCorrMax: Drive System Rotate PID Controls Proportional
   * Correction Max Limit. (percent) */
  public static final float KePID_Pct_VSN_PropCorrMax = (float) 45.0;
  
  /** KePID_Pct_VSN_PropCorrMax: Drive System Rotate PID Controls Integral
   * Correction Max Limit. (percent) */
  public static final float KePID_Pct_VSN_IntglCorrMax = (float) 50.0;
  
  /** KePID_t_VSN_PstnTgtSyncMetThrsh: Amount of time that the Position Error must
   * be held within the Error DeadBand in order for the Drive System Rotate
   * PID Controls to consider the Position Target conditions met. (seconds) */
  public static final float KePID_t_VSN_PstnTgtSyncMetThrsh = (float) 0.200;

  



  /** KnPID_Deg_NAV_FdFwdErrAxis: Angle Error Axis for the Robot
    * Rotation Control FeedFoward Correction Term for PI
    * Control of Gyro Tracking (Degrees). */
    public static final float KnPID_Deg_NAV_FdFwdErrAxis[] = new float[] 
    {
      (float)0,
      (float)5,
      (float)10,
      (float)15,
      (float)30,
      (float)45,
      (float)60,
      (float)75,
      (float)90,
      (float)120
   };	  
   

/** KtPID_Pct_NAV_FdFwdCorr: Robot Rotation Control FeedFoward
 * Correction Term for PI Control of Gyro Tracking (Degrees). */
public static final float KtPID_Pct_NAV_FdFwdCorr[] = new float[] 
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
 
 
 /** KePID_Deg_NAV_PosErrDB: Drive System Rotate Position Error DeadBand (degree). */
 public static final float KePID_Deg_NAV_PosErrDB = (float) 1.0;

 /** KePID_Deg_NAV_IntglErrDsblMin: Drive System Rotate Absolute Error Threshold
  * outside of which the Integral Correction will be disabled.  Do not start
  * applying Integral Correction until within this band. (degree) */
 public static final float KePID_Deg_NAV_IntglErrDsblMin = (float) 30.0;
 
 /** KePID_K_NAV_PropGx: Drive System Rotate PID Controls Proportional Gain. */
 public static final float KePID_K_NAV_PropGx = (float) 2.0;
 
 /** KePID_K_NAV_IntglGx: Drive System Rotate PID Controls Integral Gain. */
 public static final float KePID_K_NAV_IntglGx = (float) 1.0;

 /** KePID_Pct_NAV_PropCorrMax: Drive System Rotate PID Controls Proportional
  * Correction Max Limit. (percent) */
 public static final float KePID_Pct_NAV_PropCorrMax = (float) 45.0;
 
 /** KePID_Pct_NAV_PropCorrMax: Drive System Rotate PID Controls Integral
  * Correction Max Limit. (percent) */
 public static final float KePID_Pct_NAV_IntglCorrMax = (float) 40.0;
 
 /** KePID_t_NAV_PstnTgtSyncMetThrsh: Amount of time that the Position Error must
  * be held within the Error DeadBand in order for the Drive System Rotate
  * PID Controls to consider the Position Target conditions met. (seconds) */
 public static final float KePID_t_NAV_PstnTgtSyncMetThrsh = (float) 0.200;


}
