package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.commands.*;
import frc.robot.models.EncoderConversions;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

enum DrivePosition {
  FrontLeft, FrontRight, RearLeft, RearRight
}

public class Drives extends Subsystem {

  WPI_TalonSRX[] EncoderedDrives = { new WPI_TalonSRX(RobotMap.EncoderDriveAddresses[0]),
      new WPI_TalonSRX(RobotMap.EncoderDriveAddresses[1]), new WPI_TalonSRX(RobotMap.EncoderDriveAddresses[2]),
      new WPI_TalonSRX(RobotMap.EncoderDriveAddresses[3]) };

  WPI_TalonSRX[] FollowerDrives = { new WPI_TalonSRX(RobotMap.FollowerDriveAddresses[0]),
      new WPI_TalonSRX(RobotMap.FollowerDriveAddresses[1]), new WPI_TalonSRX(RobotMap.FollowerDriveAddresses[2]),
      new WPI_TalonSRX(RobotMap.FollowerDriveAddresses[3]) };

  public Drives() {
    setFollowers();
    talonSettings();
  }

  /********************************************
   * Drive Settings
   */

  private void talonSettings() {

    final boolean[] dInv = {true, true, true, true}; // use this to invert the drives safely
    final boolean[] sInv = {false, true, false, true}; // use this to invert the sensors safely
    for (var i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].configFactoryDefault();
      EncoderedDrives[i].configMotionCruiseVelocity(9000);
      EncoderedDrives[i].configMotionAcceleration(9000);
      EncoderedDrives[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      EncoderedDrives[i].config_kF(0, 0.073);
      EncoderedDrives[i].config_kP(0, 0.2);
      EncoderedDrives[i].config_kI(0, 0.00007);
      EncoderedDrives[i].config_kD(0, 0.0013);
      EncoderedDrives[i].setInverted(dInv[i]);
      FollowerDrives[i].setInverted(dInv[i]);
      EncoderedDrives[i].setSensorPhase(sInv[i]);
    }
  }

  private void setFollowers() {
    for (var i = 0; i < EncoderedDrives.length; i++) {
      FollowerDrives[i].set(ControlMode.Follower, EncoderedDrives[i].getDeviceID());
    }
  }

  /***************
   * Get raw Velocities
   * 
   * @return In Ticks per 100ms
   */
  public double[] rawVelocities() {
    var Velocities = new double[EncoderedDrives.length];
    for (int i = 0; i < EncoderedDrives.length; i++) {
      Velocities[i] = EncoderedDrives[i].getSelectedSensorVelocity();
    }

    return Velocities;
  }

  /**********************
   * Get Raw Positions
   * 
   * @return In Ticks
   */
  public double[] rawPosition() {
    var Positions = new double[EncoderedDrives.length];
    for (int i = 0; i < EncoderedDrives.length; i++) {
      Positions[i] = EncoderedDrives[i].getSelectedSensorPosition();
    }
    return Positions;
  }

  public int[] rawiPosition() {
    var Positions = new int[EncoderedDrives.length];
    for (int i = 0; i < EncoderedDrives.length; i++) {
      Positions[i] = EncoderedDrives[i].getSelectedSensorPosition();
    }
    return Positions;
  }

  public void resetPosition() {
    for (int i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].setSelectedSensorPosition(0);
    }
  }

  /****************************************************************
   * Drive Functions
   * 
   * 
   * Includes all functions required to drive the robot
   * 
   * 
   */
  public void DriveInVoltage(double F, double L, double R) {
    var y = deadzone(F);
    var x = deadzone(L);
    var r = deadzone(R);

    double wheelSpeeds[] = { y + x + r, y - x - r, y - x + r, y + x - r };
    wheelSpeeds = normalize(scale(wheelSpeeds, 1.0));

    for (int i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].set(ControlMode.PercentOutput, wheelSpeeds[i]);
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

    for (var i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].set(ControlMode.MotionMagic, Distance[i]);
    }

  }

  public void MMControlTest(int Distance) {
    EncoderedDrives[0].set(ControlMode.MotionMagic, Distance);
    EncoderedDrives[1].set(ControlMode.MotionMagic, Distance);
    EncoderedDrives[2].set(ControlMode.MotionMagic, Distance);
    EncoderedDrives[3].set(ControlMode.MotionMagic, Distance);
  }

  public void MMControl(double Distance) {
    for (var i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].set(ControlMode.MotionMagic, EncoderConversions.inToTicks(Distance));
    }
  }


  public int[] getClosedLoopError() {
    int[] val = new int[4];
    for (int i = 0; i < EncoderedDrives.length; i++) {
      val[i] = EncoderedDrives[i].getClosedLoopError();
    }
    return val;
  }

  public boolean[] mmIsDone() {
    boolean[] flags = new boolean[4];
    for (int i = 0; i < EncoderedDrives.length; i++) {
      flags[i] = EncoderedDrives[i].getClosedLoopError() < 500;
    }
    return flags;
  }

  public void setSafety(boolean set) {
    for (int i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].setSafetyEnabled(set);
    }

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CT_DrvCntrl());
  }
}