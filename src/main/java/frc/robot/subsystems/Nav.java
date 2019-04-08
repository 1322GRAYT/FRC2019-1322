package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Robot;
import frc.robot.calibrations.K_Nav;
import frc.robot.calibrations.K_Drive;
import frc.robot.calibrations.K_System;


public class Nav extends Subsystem {
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private double VeNAV_Deg_GyroAngle;      // double: degree heading

	// Drive System Encoders/Wheels
	// Idx [0] - FL: Front Left		
	// Idx [1] - FR: Front Right	
	// Idx [2] - RL: Rear Left	
  // Idx [3] - RR: Rear Right
  private double VaNAV_cnt_EncdrCnt[]  = new double[4];  // (counts)
	private double VaNAV_f_EncdrVelRaw[] = new double[4];  // (counts/100msec)
	private double VaNAV_f_EncdrVel[] = new double[4];     // (counts/sec)
	private double VaNAV_n_EncdrRPM[] = new double[4];     // (rpm)
	private double VaNAV_n_WhlRPM[]   = new double[4];     // (rpm)
	private double VaNAV_v_WhlVel[]   = new double[4];     // (feet/sec)

  private boolean VeNAV_b_CL_TgtRqstActv;  // boolean
  private boolean VeNAV_b_CL_DrvRqstActv;  // boolean
  private boolean VeNAV_b_DrvAutoCmdActv;  // boolean
  private boolean VeNAV_b_DrvStkRqstActv;  // boolean
  private double VeNAV_r_NormPwrDrvrLtY;   // double: normalized power - Drivers Stick Left - Y Axis
  private double VeNAV_r_NormPwrDrvrLtX;   // double: normalized power - Drivers Stick Left - X Axis
  private double VeNAV_r_NormPwrDrvrRtX;   // double: normalized power - Drivers Stick Right - X Axis
  private double VeNAV_r_NormPwrCmdLong;   // double: normalized power - longitudinal
  private double VeNAV_r_NormPwrCmdLat;    // double: normalized power - lateral
  private double VeNAV_r_NormPwrCmdRot;    // double: normalized power - rotational


  public Nav() {
    /* Empty Constructor */
    }


  /*****************************************************
   * Public Interfaces: Drive Encoder and Wheel Speed  *
   *****************************************************/

  /**
   * Method: getNAV_cnt_EncdrCnt - Drive Motor Encoder Counts
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - counts
   */
  public double getNAV_cnt_EncdrCnt(int idx)  {
    return VaNAV_cnt_EncdrCnt[idx];   
  }

  /**
   * Method: getNAV_f_EncdrVelRaw - Drive Motor Encoder Raw Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - counts/100ms
   */
  public double getNAV_f_EncdrVelRaw(int idx)  {
    return VaNAV_f_EncdrVelRaw[idx];   
  }

  /**
   * Method: getNAV_f_EncdrVel - Drive Motor Encoder Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - counts/sec
   */
  public double getNAV_f_EncdrVel(int idx)  {
    return VaNAV_f_EncdrVel[idx];   
  }

  /**
   * Method: getNAV_n_EncdrRPM - Drive Motor Encoder Angular Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - rpm
   */
  public double getNAV_n_EncdrRPM(int idx)  {
    return VaNAV_n_EncdrRPM[idx];   
  }


  /**
   * Method: getNAV_n_WhlRPM - Drive Wheel Angular Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - rpm
   */
  public double getNAV_n_WhlRPM(int idx)  {
    return VaNAV_n_WhlRPM[idx];   
  }

  /**
   * Method: getNAV_v_WhlVel - Drive Wheel Lineal Velocity
   * @param: int - encoder index: 0-3 (FL, FR, RL, RR)  
   * @return: double - inches/sec
   */
  public double getNAV_v_WhlVel(int idx)  {
    return VaNAV_v_WhlVel[idx];   
  }


  /**************************************************
   ** Public Interfaces: Nav System Control Task   **
   **                                              **
   **************************************************/

  public boolean getNAV_CL_TgtRqstActv() {
    return(VeNAV_b_CL_TgtRqstActv);  
  }

  public void setNAV_CL_TgtRqstActv(boolean RqstSt) {
    VeNAV_b_CL_TgtRqstActv = RqstSt;
  }

  public boolean getNAV_CL_DrvRqstActv() {
    return(VeNAV_b_CL_DrvRqstActv);  
  }

  public void setNAV_CL_DrvRqstActv(boolean RqstSt) {
    VeNAV_b_CL_DrvRqstActv = RqstSt;
  }

  public boolean getNAV_b_DrvAutoCmdActv() {
    return(VeNAV_b_DrvAutoCmdActv);  
  }

  public void setNAV_b_DrvAutoCmdActv(boolean RqstSt) {
    VeNAV_b_DrvAutoCmdActv = RqstSt;
  }

  public boolean getNAV_b_DrvStkRqstActv() {
    return(VeNAV_b_DrvStkRqstActv);  
  }

  public void setNAV_b_DrvStkRqstActv(boolean RqstSt) {
    VeNAV_b_DrvStkRqstActv = RqstSt;
  }

  public double getNAV_r_NormPwrDrvrLtY() {
    return(VeNAV_r_NormPwrDrvrLtY);
  }

  public void setNAV_r_NormPwrDrvrLtY(double StkInp) {
    VeNAV_r_NormPwrDrvrLtY = StkInp;
  }
 
  public double getNAV_r_NormPwrDrvrLtX() {
    return(VeNAV_r_NormPwrDrvrLtX);
  }

  public void setNAV_r_NormPwrDrvrLtX(double StkInp) {
    VeNAV_r_NormPwrDrvrLtX = StkInp;
  }
  
  public double getNAV_r_NormPwrDrvrRtX() {
    return(VeNAV_r_NormPwrDrvrRtX);
  }

  public void setNAV_r_NormPwrDrvrRtX(double StkInp) {
    VeNAV_r_NormPwrDrvrRtX = StkInp;
  }

  public double getNAV_r_NormPwrCmdLong() {
    return(VeNAV_r_NormPwrCmdLong);
  }

  public double getNAV_r_NormPwrCmdLat() {
    return(VeNAV_r_NormPwrCmdLat);
  }
 
  public double getNAV_r_NormPwrCmdRot() {
    return(VeNAV_r_NormPwrCmdRot);
  }



  /******************************************
   ** Public Class Methods                 **
   **                                      **
   ******************************************/

/** Method: mngNAV_InitCntrl - Manages the Controller Initialization
  * Tasks that should be run when the robot is initialized or reset.
  */	
  public void mngNAV_InitCntrl() {
    perfmNAV_GyroCal();
    rstNAV_DrvCmndArb();
  }

/** Method: mngNAV_CmndSysTsk1 - Manages the Navigation Control
  * Command System Periodic Task 1 (runs First).
  */	
  public void mngNAV_CmndSysTsk1() {
    updNAV_GyroAngle();
    updNAV_DrvWhlData();
  }

/** Method: mngNAV_CmndSysTsk2 - Manages the Navigation Control
  * Command System Periodic Task 2 (runs Second).
  */	
  public void mngNAV_CmndSysTsk2() {
    cntrlNAV_DrvCmndArb();
  }


/** Method: rstNAV_GyroAngle - Resets the current heading of the
  * Gyrometer as the 0 degree mark.
  */	
  public void rstNAV_GyroAngle() {
    gyro.reset();
  }


/** Method: rstNAV_GyroAngle - Provides access to the current value
  * of the Gyrometer Angle reading (Positive is Clockwise: [0 to 360) )
  */	
  public double getNAV_GyroAngle() {
    return(VeNAV_Deg_GyroAngle);
  }
  

  /** Method: cvrtAngToLinSpd - Calculates the Linear 
   *  Speed of the Robot from the Angular Wheel Speed
   *  when moving directly forward or rearward.
   *  @param: Wheel Angular Speed (rpm)
   *  @return: Robot Linear Speed (inch/sec) */	
   public static float cvrtAngToLinSpd(float SpdWhl) {
    return ((float)((K_Nav.KeNAV_l_DistPerRevWhl * SpdWhl)/(float)60));
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
   
    WhlRevs   = (double)(DistInch / K_Nav.KeNAV_l_DistPerRevWhl);	 
    EncdrRevs = WhlRevs * (double)K_Nav.KeNAV_r_EncdrToWhl;	 
    EncdrCnts = EncdrRevs * (double)K_Nav.KeNAV_Cnt_PlsPerRevEncdr;
   
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

    EncdrRevs = (double)EncdrCnts / (double)K_Nav.KeNAV_Cnt_PlsPerRevEncdr;
    WhlRevs   = EncdrRevs / (double)K_Nav.KeNAV_r_EncdrToWhl;	 
    DistInch  = WhlRevs * (double)K_Nav.KeNAV_l_DistPerRevWhl;	 
  
    return DistInch;
  }



  /******************************************
   ** Private Class Methods                **
   **                                      **
   ******************************************/

/** Method: perfmNAV_GyroCal - Performs the Calibration of the
  * Gyrometer to set the proper home / oridinal positions.
  */	
  private void perfmNAV_GyroCal() {
    gyro.calibrate();
  }


/** Method: updNAV_GyroAngle - Provides access to the current value
  * of the Gyrometer Angle reading (Positive is Clockwise: [0 to 360) )
  */	
  private void updNAV_GyroAngle() {
    VeNAV_Deg_GyroAngle = gyro.getAngle();
  }


  /** Method: updNAV_DrvWhlData - Updates the derived Drive Motor Encoder and 
   *  wheel data at a periodic rate so that only one set of Talon Calls for Sensor
   *  Positions and Velocity per loop is done to minimize throughput.  Each Call to
   *  EncodedDrives requires a CAN Request and Response.
   */
  private void updNAV_DrvWhlData() {
    int idx;
//  double tempCnt[] = new double[4];      // (counts)
//  double tempRPM[] = new double[4];      // (counts/100msec)

    /* Drive Speed Inputs */
//  tempCnt = Robot.DRIVES.EncodedDrives.getSelectedSensorPosition();  // counts
//  tempRPM = Robot.DRIVES.EncodedDrives.getSelectedSensorVelocity();  // counts/100msec
  
    for (idx=0; idx<4; idx++) {
//    VaNAV_cnt_EncdrCnt[idx]  = Math.abs(tempCnt[idx]);        // Counts
//    VaNAV_f_EncdrVelRaw[idx] = Math.abs(tempRPM[idx]);        // counts/100ms
      VaNAV_cnt_EncdrCnt[idx]  = Robot.DRIVES.EncodedDrives[idx].getSelectedSensorPosition();  // Counts
      VaNAV_f_EncdrVelRaw[idx] = Robot.DRIVES.EncodedDrives[idx].getSelectedSensorVelocity();  // counts/100ms
      VaNAV_f_EncdrVel[idx]    = VaNAV_f_EncdrVelRaw[idx] * 10; // counts/sec 
      VaNAV_n_EncdrRPM[idx]    = (VaNAV_f_EncdrVel[idx]/K_Nav.KeNAV_Cnt_PlsPerRevEncdr)*(60); // rpm  (1 rpm = 600 rev/100 rpm)
      VaNAV_n_WhlRPM[idx]      = VaNAV_n_EncdrRPM[idx]/K_Nav.KeNAV_r_EncdrToWhl;              // rpm
      VaNAV_v_WhlVel[idx]      = (VaNAV_n_WhlRPM[idx]*K_Nav.KeNAV_l_DistPerRevWhl)/60;        // inches/sec
    }

    if (K_System.KeSYS_b_DebugEnblDrv == true) {
      Robot.DASHBOARD.updateSmartDashDrvData();
    }
  }

  
  /** Method: cntrlNAV_DrvCmndArb - Controls the Arbitration of the Commands
    * of the Drive System Motor Command based on the different sources of
    * Navigation Autonomous and Driver Tele-Operation Controls.
    */
    private void cntrlNAV_DrvCmndArb() {
      double LeNAV_r_NormPwrCmdLong, LeNAV_r_NormPwrCmdLat, LeNAV_r_NormPwrCmdRot;
      boolean LeNAV_b_DrvrStkCntrl = false;
      boolean LeNAV_b_SetSafe = false;

      if ((Math.abs(getNAV_r_NormPwrDrvrLtY()) >= K_Drive.KeDRV_r_DB_InpLong) ||
          (Math.abs(getNAV_r_NormPwrDrvrLtX()) >= K_Drive.KeDRV_r_DB_InpLat) ||
          (Math.abs(getNAV_r_NormPwrDrvrRtX()) >= K_Drive.KeDRV_r_DB_InpRot)) {
        setNAV_b_DrvStkRqstActv(LeNAV_b_DrvrStkCntrl);
      }

      /* Drive Control Arbitration */
      if ((getNAV_CL_TgtRqstActv() == true) &&
          (getNAV_CL_DrvRqstActv() == true)) {
        LeNAV_r_NormPwrCmdLong = 0.0;
        LeNAV_r_NormPwrCmdLat =  0.0;
        LeNAV_r_NormPwrCmdRot =  Robot.PID.getPID_r_PwrCmndNorm();
      }
      else if ((getNAV_CL_TgtRqstActv() == true) &&
               (getNAV_CL_DrvRqstActv() == false)) {
        LeNAV_r_NormPwrCmdLong = K_Nav.KeNAV_r_CL_NormPwrLong;
        LeNAV_r_NormPwrCmdLat =  Robot.PID.getPID_r_PwrCmndNorm() * K_Nav.KeNAV_r_CL_ScalarRotToLat;
        LeNAV_r_NormPwrCmdRot =  0.0;                 
      }
      else if (getNAV_b_DrvStkRqstActv()) {
        LeNAV_r_NormPwrCmdLong = getNAV_r_NormPwrDrvrLtY();
        LeNAV_r_NormPwrCmdLat =  getNAV_r_NormPwrDrvrLtX();
        LeNAV_r_NormPwrCmdRot =  getNAV_r_NormPwrDrvrRtX();
      }
      else { /* (getDRV_CL_TgtRqstActv() == false) */
        /* Disable Drives */
        LeNAV_r_NormPwrCmdLong = 0.0;
        LeNAV_r_NormPwrCmdLat =  0.0;
        LeNAV_r_NormPwrCmdRot =  0.0;
        LeNAV_b_SetSafe = true;
      }

      VeNAV_r_NormPwrCmdLong = LeNAV_r_NormPwrCmdLong;
      VeNAV_r_NormPwrCmdLat =  LeNAV_r_NormPwrCmdLat;
      VeNAV_r_NormPwrCmdRot =  LeNAV_r_NormPwrCmdRot;
      if ((K_System.KeSYS_b_CL_DrvTgtEnbl == true) && (getNAV_b_DrvAutoCmdActv() == false)) {
        Robot.DRIVES.DriveInVoltage(LeNAV_r_NormPwrCmdLong, LeNAV_r_NormPwrCmdLat, LeNAV_r_NormPwrCmdRot);
        Robot.DRIVES.setSafety(LeNAV_b_SetSafe);
      }
      if (K_System.KeSYS_b_DebugEnblCL == true) {
        Robot.DASHBOARD.updateSmartDashTgtCLData();
        Robot.DASHBOARD.updateSmartDashDrvSysData();
      }

    }


  /** Method: rstNAV_DrvCmndArb - Resets the variables of the Control
    * Arbitration of the Commands of the Drive System Motor Commands.
    */
    private void rstNAV_DrvCmndArb() {
      VeNAV_b_CL_TgtRqstActv = false;
      VeNAV_b_CL_DrvRqstActv = false;
      VeNAV_b_DrvAutoCmdActv = false;
      VeNAV_b_DrvStkRqstActv = false;
      VeNAV_r_NormPwrDrvrLtY = 0.0;
      VeNAV_r_NormPwrDrvrLtX = 0.0;
      VeNAV_r_NormPwrDrvrRtX = 0.0;
      VeNAV_r_NormPwrCmdLong = 0.0;
      VeNAV_r_NormPwrCmdLat =  0.0;
      VeNAV_r_NormPwrCmdRot =  0.0;
    }




  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CC_DrvVsnTgtDsbl());
  }
}