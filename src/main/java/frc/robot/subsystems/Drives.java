package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
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
      FollowerDrives[i].setSafetyEnabled(false);
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



  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CT_DrvCntrl());
  }
}