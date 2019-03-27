package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.calibrations.K_Drive;

enum DrivePosition {
  FrontLeft, FrontRight, RearLeft, RearRight
}

public class Drives extends Subsystem {
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
      EncodedDrives[i].configMotionCruiseVelocity(K_Drive.KDRV_n_MM_CruiseVel);
      EncodedDrives[i].configMotionAcceleration(K_Drive.KDRV_a_MM_MaxAccel);
      EncodedDrives[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      EncodedDrives[i].config_kF(0, K_Drive.KDRV_dPct_VelFeedFwdTerm);
      EncodedDrives[i].config_kP(0, K_Drive.KDRV_k_VelPropGx);
      EncodedDrives[i].config_kI(0, K_Drive.KDRV_k_VelIntglGx);
      EncodedDrives[i].config_kD(0, K_Drive.KDRV_k_VelDerivGx);
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

  /****************************************************************
   * Drive Functions
   * Includes all functions required to drive the robot
   */
  public void DriveInVoltage(double F, double L, double R) {
    var y = deadzone(F);
    var x = deadzone(L);
    var r = deadzone(R);

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

  private double deadzone(double power) {
    return Math.abs(power) > 0.2 ? power : 0.0;
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



  /** Method: updateDrvEncdrData - Updates the derived Drive Encoder data at
   *  a periodic rate so that only one set of Talon Calls for Sensor Positions
   *  and Velocity per loop is done to minimize throughput.  Each Call to
   *  EncodedDrives requires a CAN Request and Response.
   */
  public void updateDrvEncdrData() {
    int idx;
      
    /* Drive Speed Inputs */    
    for (idx=0; idx<4; idx++) {
//    VaDRV_cnt_EncdrCnt[idx]  = Math.abs(tempCnt[idx]);        // Counts
//    VaDRV_f_EncdrVelRaw[idx] = Math.abs(tempRPM[idx]);        // counts/100ms
      VaDRV_cnt_EncdrCnt[idx]  = EncodedDrives[idx].getSelectedSensorPosition();  // Counts
      VaDRV_f_EncdrVelRaw[idx] = EncodedDrives[idx].getSelectedSensorVelocity();  // counts/100ms
      VaDRV_f_EncdrVel[idx]    = VaDRV_f_EncdrVelRaw[idx] * 10; // counts/sec 
      VaDRV_n_EncdrRPM[idx]    = (VaDRV_f_EncdrVel[idx]/K_Drive.KDRV_Cnt_PlsPerRevEncdr)*(60); // rpm  (1 rpm = 600 rev/100 rpm)
      VaDRV_n_WhlRPM[idx]      = VaDRV_n_EncdrRPM[idx]/K_Drive.KDRV_r_EncdrToWhl;              // rpm
      VaDRV_v_WhlVel[idx]      = (VaDRV_n_WhlRPM[idx]*K_Drive.KDRV_l_DistPerRevWhl)/60;        // inches/sec
    }        
  }


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CT_DrvCntrl());
  }
}