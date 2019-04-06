package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.calibrations.K_Drive;
import frc.robot.calibrations.K_System;

enum DrivePosition {
  FrontLeft, FrontRight, RearLeft, RearRight
}

public class Drives extends Subsystem {
  private boolean VeDRV_b_CL_TgtRqstActv;  // boolean
  private boolean VeDRV_b_CL_DrvRqstActv;  // boolean
  private boolean VeDRV_b_DrvAutoCmdActv;  // boolean
  private boolean VeDRV_b_DrvStkRqstActv;  // boolean
  private double VeDRV_r_NormPwrDrvrLtY;   // double: normalized power - Drivers Stick Left - Y Axis
  private double VeDRV_r_NormPwrDrvrLtX;   // double: normalized power - Drivers Stick Left - X Axis
  private double VeDRV_r_NormPwrDrvrRtX;   // double: normalized power - Drivers Stick Right - X Axis
  private double VeDRV_r_NormPwrCmdLong;   // double: normalized power - longitudinal
  private double VeDRV_r_NormPwrCmdLat;    // double: normalized power - lateral
  private double VeDRV_r_NormPwrCmdRot;    // double: normalized power - rotational

	// Drive System Encoders/Wheels
	// Idx [0] - FL: Front Left		
	// Idx [1] - FR: Front Right	
	// Idx [2] - RL: Rear Left	
  // Idx [3] - RR: Rear Right
  private double VaDRV_cnt_EncdrCnt[]  = new double[4];  // (counts)
	private double VaDRV_f_EncdrVelRaw[] = new double[4];  // (counts/100msec)
	private double VaDRV_f_EncdrVel[] = new double[4];     // (counts/sec)
	private double VaDRV_n_EncdrRPM[] = new double[4];     // (rpm)
	private double VaDRV_n_WhlRPM[]   = new double[4];     // (rpm)
	private double VaDRV_v_WhlVel[]   = new double[4];     // (feet/sec)


  WPI_TalonSRX[] EncodedDrives = { new WPI_TalonSRX(RobotMap.EncoderDriveAddresses[0]),
      new WPI_TalonSRX(RobotMap.EncoderDriveAddresses[1]), new WPI_TalonSRX(RobotMap.EncoderDriveAddresses[2]),
      new WPI_TalonSRX(RobotMap.EncoderDriveAddresses[3]) };

  WPI_TalonSRX[] FollowerDrives = { new WPI_TalonSRX(RobotMap.FollowerDriveAddresses[0]),
      new WPI_TalonSRX(RobotMap.FollowerDriveAddresses[1]), new WPI_TalonSRX(RobotMap.FollowerDriveAddresses[2]),
      new WPI_TalonSRX(RobotMap.FollowerDriveAddresses[3]) };

  public Drives() {
    final boolean[] dInv = {true, true, true, true}; // use this to invert the drives safely
    final boolean[] sInv = {false, true, false, true}; // use this to invert the sensors safely
    for (var i = 0; i < EncodedDrives.length; i++) {
      EncodedDrives[i].configFactoryDefault();
      EncodedDrives[i].configMotionCruiseVelocity(K_Drive.KeDRV_n_MM_CruiseVel);
      EncodedDrives[i].configMotionAcceleration(K_Drive.KeDRV_a_MM_MaxAccel);
      EncodedDrives[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      EncodedDrives[i].config_kF(0, K_Drive.KeDRV_dPct_VelFeedFwdTerm);
      EncodedDrives[i].config_kP(0, K_Drive.KeDRV_k_VelPropGx);
      EncodedDrives[i].config_kI(0, K_Drive.KeDRV_k_VelIntglGx);
      EncodedDrives[i].config_kD(0, K_Drive.KeDRV_k_VelDerivGx);
      EncodedDrives[i].setInverted(dInv[i]);
      FollowerDrives[i].set(ControlMode.Follower, EncodedDrives[i].getDeviceID());
      FollowerDrives[i].setInverted(dInv[i]);
      EncodedDrives[i].setSensorPhase(sInv[i]);
    }
  }

  /***************
   * Get raw Velocities
   * 
   * @return In Ticks per 100ms
   */
  public double[] rawVelocities() {
    var Velocities = new double[EncodedDrives.length];
    for (int i = 0; i < EncodedDrives.length; i++) {
      Velocities[i] = EncodedDrives[i].getSelectedSensorVelocity();
    }

    return Velocities;
  }

  /**********************
   * Get Raw Positions
   * 
   * @return In Ticks
   */
  public double[] rawPosition() {
    var Positions = new double[EncodedDrives.length];
    for (int i = 0; i < EncodedDrives.length; i++) {
      Positions[i] = EncodedDrives[i].getSelectedSensorPosition();
    }
    return Positions;
  }

  public void resetPosition() {
    for (int i = 0; i < EncodedDrives.length; i++) {
      EncodedDrives[i].setSelectedSensorPosition(0);
    }
  }


  /**************************************
   ** Public Interfaces Control Task   **
   **                                  **
   **************************************/

  public boolean getDRV_CL_TgtRqstActv() {
    return(VeDRV_b_CL_TgtRqstActv);  
  }

  public void setDRV_CL_TgtRqstActv(boolean RqstSt) {
    VeDRV_b_CL_TgtRqstActv = RqstSt;
  }

  public boolean getDRV_CL_DrvRqstActv() {
    return(VeDRV_b_CL_DrvRqstActv);  
  }

  public void setDRV_CL_DrvRqstActv(boolean RqstSt) {
    VeDRV_b_CL_DrvRqstActv = RqstSt;
  }

  public boolean getDRV_b_DrvAutoCmdActv() {
    return(VeDRV_b_DrvAutoCmdActv);  
  }

  public void setDRV_b_DrvAutoCmdActv(boolean RqstSt) {
    VeDRV_b_DrvAutoCmdActv = RqstSt;
  }

  public boolean getDRV_b_DrvStkRqstActv() {
    return(VeDRV_b_DrvStkRqstActv);  
  }

  public void setDRV_b_DrvStkRqstActv(boolean RqstSt) {
    VeDRV_b_DrvStkRqstActv = RqstSt;
  }

  public double getDRV_r_NormPwrDrvrLtY() {
    return(VeDRV_r_NormPwrDrvrLtY);
  }

  public void setDRV_r_NormPwrDrvrLtY(double StkInp) {
    VeDRV_r_NormPwrDrvrLtY = StkInp;
  }
 
  public double getDRV_r_NormPwrDrvrLtX() {
    return(VeDRV_r_NormPwrDrvrLtX);
  }

  public void setDRV_r_NormPwrDrvrLtX(double StkInp) {
    VeDRV_r_NormPwrDrvrLtX = StkInp;
  }
  
  public double getDRV_r_NormPwrDrvrRtX() {
    return(VeDRV_r_NormPwrDrvrRtX);
  }

  public void setDRV_r_NormPwrDrvrRtX(double StkInp) {
    VeDRV_r_NormPwrDrvrRtX = StkInp;
  }

  public double getDRV_r_NormPwrCmdLong() {
    return(VeDRV_r_NormPwrCmdLong);
  }

  public double getDRV_r_NormPwrCmdLat() {
    return(VeDRV_r_NormPwrCmdLat);
  }
 
  public double getDRV_r_NormPwrCmdRot() {
    return(VeDRV_r_NormPwrCmdRot);
  }


  /************************************************
   ** Periodic Drive Control Strategy Tasks      **
   **                                            **
   ************************************************/
   
  public void mngDRV_CntrlPeriodic() {
    double LeDRV_r_NormPwrCmdLong, LeDRV_r_NormPwrCmdLat, LeDRV_r_NormPwrCmdRot;
    boolean LeDRV_b_DrvrStkCtrl = false;
    boolean LeDRV_b_SetSafe = false;

    if ((Math.abs(getDRV_r_NormPwrDrvrLtY()) >= K_Drive.KeDRV_r_DB_InpLong) ||
        (Math.abs(getDRV_r_NormPwrDrvrLtX()) >= K_Drive.KeDRV_r_DB_InpLat) ||
        (Math.abs(getDRV_r_NormPwrDrvrRtX()) >= K_Drive.KeDRV_r_DB_InpRot)) {
      setDRV_b_DrvStkRqstActv(LeDRV_b_DrvrStkCtrl);
    }

    /* Drive Control Arbitration */
    if ((getDRV_CL_TgtRqstActv() == true) &&
        (getDRV_CL_DrvRqstActv() == true)) {
          LeDRV_r_NormPwrCmdLong = 0.0;
          LeDRV_r_NormPwrCmdLat =  0.0;
          LeDRV_r_NormPwrCmdRot =  Robot.PIDF.VePID_r_PwrCmndNorm;
    }
    else if ((getDRV_CL_TgtRqstActv() == true) &&
             (getDRV_CL_DrvRqstActv() == false)) {
              LeDRV_r_NormPwrCmdLong = K_Drive.KeDRV_r_CL_NormPwrLong;
              LeDRV_r_NormPwrCmdLat =  Robot.PIDF.VePID_r_PwrCmndNorm * K_Drive.KeDRV_r_CL_ScalarRotToLat;
              LeDRV_r_NormPwrCmdRot =  0.0;                 
    }
    else if (getDRV_b_DrvStkRqstActv()) {
      LeDRV_r_NormPwrCmdLong = getDRV_r_NormPwrDrvrLtY();
      LeDRV_r_NormPwrCmdLat =  getDRV_r_NormPwrDrvrLtX();
      LeDRV_r_NormPwrCmdRot =  getDRV_r_NormPwrDrvrRtX();
    }
    else { /* (getDRV_CL_TgtRqstActv() == false) */
      /* Disable Drives */
      LeDRV_r_NormPwrCmdLong = 0.0;
      LeDRV_r_NormPwrCmdLat =  0.0;
      LeDRV_r_NormPwrCmdRot =  0.0;
      LeDRV_b_SetSafe = true;
    }

    VeDRV_r_NormPwrCmdLong = LeDRV_r_NormPwrCmdLong;
    VeDRV_r_NormPwrCmdLat =  LeDRV_r_NormPwrCmdLat;
    VeDRV_r_NormPwrCmdRot =  LeDRV_r_NormPwrCmdRot;
    if ((K_System.KeSYS_b_CL_DrvTgtEnbl == true) && (getDRV_b_DrvAutoCmdActv() == false)) {
      DriveInVoltage(LeDRV_r_NormPwrCmdLong, LeDRV_r_NormPwrCmdLat, LeDRV_r_NormPwrCmdRot);
      setSafety(LeDRV_b_SetSafe);
    }
    if (K_System.KeSYS_b_DebugEnblCL == true) {
      Robot.DASHBOARD.updateSmartDashTgtCLData();
      Robot.DASHBOARD.updateSmartDashDrvSysData();
    }
  }


  public void rstDRV_CntrlPeriodic() {
    VeDRV_b_CL_TgtRqstActv = false;
    VeDRV_b_CL_DrvRqstActv = false;
    VeDRV_r_NormPwrCmdLong = 0.0;
    VeDRV_r_NormPwrCmdLat =  0.0;
    VeDRV_r_NormPwrCmdRot =  0.0;
  }


  /**************************************
   ** Motion Magic Drive Control       **
   **                                  **
   **************************************/
  
  /****************************************************************
   * Drive Functions
   * Includes all functions required to drive the robot
   */
  public void DriveInVoltage(double Longitudinal, double Lateral, double Rotate) {
    var y = deadzone(Longitudinal, K_Drive.KeDRV_r_DB_InpLong);
    var x = deadzone(Lateral, K_Drive.KeDRV_r_DB_InpLat);
    var r = deadzone(Rotate, K_Drive.KeDRV_r_DB_InpRot);

    double wheelSpeeds[] = { y + x + r, y - x - r, y - x + r, y + x - r };
    wheelSpeeds = normalize(scale(wheelSpeeds, 1.0));

    for (int i = 0; i < EncodedDrives.length; i++) {
      EncodedDrives[i].set(ControlMode.PercentOutput, wheelSpeeds[i]);
    }
  }

  private static double[] scale(double wheelSpeeds[], double scaleFactor) {
    for (var i = 0; i < wheelSpeeds.length; i++) {
      wheelSpeeds[i] *= scaleFactor;
    }
    return wheelSpeeds;
  }

  private static double[] normalize(double wheelSpeeds[]) {
    double maxMagnitude = wheelSpeeds[0];
    for (double p : wheelSpeeds) {
      maxMagnitude = Math.max(maxMagnitude, p);
    }

    if (maxMagnitude > 1.0) {
      for (var i = 0; i < wheelSpeeds.length; i++) {
        wheelSpeeds[i] /= maxMagnitude;
      }
    }
    return wheelSpeeds;
  }

  /************************************
   * Deadzone for using joysticks, if tolerance is within range
   */
  private double deadzone(double power , double tol) {
    return Math.abs(power) - tol > 0 ? power : 0.0;
  }

  /****************************
   * Autonomous Mode Functions
   */

  public void MMControl(int[] Distance) {
    for (var i = 0; i < EncodedDrives.length; i++) {
      EncodedDrives[i].set(ControlMode.MotionMagic, Distance[i]);
    }
  }

  public int[] getClosedLoopError() {
    int[] val = new int[4];
    for (int i = 0; i < EncodedDrives.length; i++) {
      val[i] = EncodedDrives[i].getClosedLoopError();
    }
    return val;
  }

  public boolean[] mmIsDone() {
    boolean[] flags = new boolean[4];
    for (int i = 0; i < EncodedDrives.length; i++) {
      flags[i] = EncodedDrives[i].getClosedLoopError() < 500;
    }
    return flags;
  }

  public void setSafety(boolean set) {
    for (int i = 0; i < EncodedDrives.length; i++) {
      EncodedDrives[i].setSafetyEnabled(set);
    }
  }




  /******************************************
   ** Drive Encoder and Speed Calculations **
   **                                      **
   ******************************************/

  /************************************
   * Encoder Public Interfaces        *
   ************************************/

  /**
   * Method: getDRV_cnt_EncdrCnt - Drive Motor Encoder Counts
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - counts
   */
  public double getDRV_cnt_EncdrCnt(int idx)  {
    return VaDRV_cnt_EncdrCnt[idx];   
  }

  /**
   * Method: getDRV_f_EncdrVelRaw - Drive Motor Encoder Raw Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - counts/100ms
   */
  public double getDRV_f_EncdrVelRaw(int idx)  {
    return VaDRV_f_EncdrVelRaw[idx];   
  }

  /**
   * Method: getDRV_f_EncdrVel - Drive Motor Encoder Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - counts/sec
   */
  public double getDRV_f_EncdrVel(int idx)  {
    return VaDRV_f_EncdrVel[idx];   
  }

  /**
   * Method: getDRV_n_EncdrRPM - Drive Motor Encoder Angular Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - rpm
   */
  public double getDRV_n_EncdrRPM(int idx)  {
    return VaDRV_n_EncdrRPM[idx];   
  }


  /**
   * Method: getDRV_n_WhlRPM - Drive Wheel Angular Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - rpm
   */
  public double getDRV_n_WhlRPM(int idx)  {
    return VaDRV_n_WhlRPM[idx];   
  }

  /**
   * Method: getDRV_v_WhlVel - Drive Wheel Lineal Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - inches/sec
   */
  public double getDRV_v_WhlVel(int idx)  {
    return VaDRV_v_WhlVel[idx];   
  }


  /************************************
   * Public SubSystem Methods         *
   ************************************/

  /** Method: updDRV_EncdrData - Updates the derived Drive Encoder data at
   *  a periodic rate so that only one set of Talon Calls for Sensor Positions
   *  and Velocity per loop is done to minimize throughput.  Each Call to
   *  EncodedDrives requires a CAN Request and Response.
   */
  public void updDRV_EncdrData() {
    int idx;
      
    /* Drive Speed Inputs */    
    for (idx=0; idx<4; idx++) {
//    VaDRV_cnt_EncdrCnt[idx]  = Math.abs(tempCnt[idx]);        // Counts
//    VaDRV_f_EncdrVelRaw[idx] = Math.abs(tempRPM[idx]);        // counts/100ms
      VaDRV_cnt_EncdrCnt[idx]  = EncodedDrives[idx].getSelectedSensorPosition();  // Counts
      VaDRV_f_EncdrVelRaw[idx] = EncodedDrives[idx].getSelectedSensorVelocity();  // counts/100ms
      VaDRV_f_EncdrVel[idx]    = VaDRV_f_EncdrVelRaw[idx] * 10; // counts/sec 
      VaDRV_n_EncdrRPM[idx]    = (VaDRV_f_EncdrVel[idx]/K_Drive.KeDRV_Cnt_PlsPerRevEncdr)*(60); // rpm  (1 rpm = 600 rev/100 rpm)
      VaDRV_n_WhlRPM[idx]      = VaDRV_n_EncdrRPM[idx]/K_Drive.KeDRV_r_EncdrToWhl;              // rpm
      VaDRV_v_WhlVel[idx]      = (VaDRV_n_WhlRPM[idx]*K_Drive.KeDRV_l_DistPerRevWhl)/60;        // inches/sec
    }

    if (K_System.KeSYS_b_DebugEnblDrv == true) {
      Robot.DASHBOARD.updateSmartDashDrvData();
    }

  }


  /** Method: cvrtAngToLinSpd - Calculates the Linear 
   *  Speed of the Robot from the Angular Wheel Speed
   *  when moving directly forward or rearward.
   *  @param: Wheel Angular Speed (rpm)
   *  @return: Robot Linear Speed (inch/sec) */	
   public static float cvrtAngToLinSpd(float SpdWhl) {
    return ((float)((K_Drive.KeDRV_l_DistPerRevWhl * SpdWhl)/(float)60));
  }
  

  /** Method: cvrtDistToCnts - Calculates the nominal number 
   *  of Drive encoder counts (cnts) that would be registered if
   *  the the Drive Wheel traveled forward/backward the
   *  desired distance given (inches).
   *  @param: Desired Distance (inches)
   *  @return: Encoder Counts (cnts) */
  public static double cvrtDistToCnts(float DistInch) {
    double WhlRevs;
    double EncdrRevs;
    double EncdrCnts;
   
    WhlRevs   = (double)(DistInch / K_Drive.KeDRV_l_DistPerRevWhl);	 
    EncdrRevs = WhlRevs * (double)K_Drive.KeDRV_r_EncdrToWhl;	 
    EncdrCnts = EncdrRevs * (double)K_Drive.KeDRV_Cnt_PlsPerRevEncdr;
   
    return EncdrCnts;
  }


  /** Method: cvrtCntsToDist - Calculates the nominal distance
   *  that would be/was travelled in inches based on the number
   *  of encoder counts that were registered by the the Drive Wheel
   *  encoder (cnts).
   *  @param: Encoder Counts (cnts) 
   *  @return: Desired Distance (inches) */
  public static double cvrtCntsToDist(int EncdrCnts) {
    double EncdrRevs;
    double WhlRevs;
    double DistInch;

    EncdrRevs = (double)EncdrCnts / (double)K_Drive.KeDRV_Cnt_PlsPerRevEncdr;
    WhlRevs   = EncdrRevs / (double)K_Drive.KeDRV_r_EncdrToWhl;	 
    DistInch  = WhlRevs * (double)K_Drive.KeDRV_l_DistPerRevWhl;	 
  
    return DistInch;
  }



  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CT_DrvCntrl());
  }
}