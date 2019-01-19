package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

enum DrivePosition {
  FrontLeft, FrontRight, RearLeft, RearRight
}

public class Drives extends Subsystem {
  TalonSRX[] EncoderedDrives = { new TalonSRX(RobotMap.EncoderDriveAddresses[0]),
      new TalonSRX(RobotMap.EncoderDriveAddresses[1]), new TalonSRX(RobotMap.EncoderDriveAddresses[2]),
      new TalonSRX(RobotMap.EncoderDriveAddresses[3]) };

  TalonSRX[] FolowerDrives = { new TalonSRX(RobotMap.FolowerDriveAddresses[0]),
      new TalonSRX(RobotMap.FolowerDriveAddresses[1]), new TalonSRX(RobotMap.FolowerDriveAddresses[2]),
      new TalonSRX(RobotMap.FolowerDriveAddresses[3]) };

  public Drives() {
    setFollowers();
    setEncoders();
  }

  private void setFollowers() {
    for (var i = 0; i < EncoderedDrives.length; i++) {
      FolowerDrives[i].set(ControlMode.Follower, EncoderedDrives[i].getDeviceID());
    }
  }

  private void setEncoders() {
    for (var i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }
  }

  /************
   * Unit Conversions
   * 
   * inToTicks converts linear inches to encoder ticks For use to read encoder
   * counts to ticksToIn converts encoder ticks to linear inches
   */
  private final int countsPerCycle = 200 * 4; // 200 ticks at 4x encoder (4x Encoder counts Up and Down)
  private final double diaOfWheel = 4; // in inches
  private final double circOfWheel = Math.PI * diaOfWheel; // D * PI
  private final double gearRatio = (54 / 28) * (22 / 12); // 28 to 54 to 12 to 22

  public int inToTicks(double Inches) {
    return (int) (Inches * (gearRatio * countsPerCycle / circOfWheel));
  }

  public double ticksToIn(int Ticks) {
    return ((double) Ticks) / (gearRatio * countsPerCycle / circOfWheel);
  }

  /***************
   * Get raw Velocities
   * 
   * @return In Ticks per 10ms
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

  public void resetPosition() {
    for (int i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].setSelectedSensorPosition(0);
    }
  }

  /*************************************
   * Drive Functions
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

  public void MMControl(int Distance) {

    for (var i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].set(ControlMode.MotionMagic, Distance);
    }

  }

  public void MMControl(double Distance) {
    for (var i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].set(ControlMode.MotionMagic, inToTicks(Distance));
    }
  }

  private int[] relativePosition = new int[4];

  public void setRelativePosition() {
    for (var i = 0; i < relativePosition.length; i++) {
      relativePosition[i] = EncoderedDrives[i].getSelectedSensorPosition();
    }
  }

  /*******************************************
   * For Moving a relative to the current position
   */
  public void RelativeMotionMagic(int x, int y) {
    int pHolder = (x != 0 ? x : y);

    int[] motorDistance = { (int) Math.signum(x) * pHolder, (int) Math.signum(-x) * pHolder,
        (int) Math.signum(-x) * pHolder, (int) Math.signum(x) * pHolder }; // Determine which direction the robot needs
                                                                           // to go

    for (int i = 0; i < EncoderedDrives.length; i++) {
      EncoderedDrives[i].set(ControlMode.MotionMagic,
          EncoderedDrives[i].getSelectedSensorPosition() + motorDistance[i]);
    }
  }

  public boolean[] mmIsDone() {
    boolean[] flags = new boolean[4];
    for (int i = 0; i < EncoderedDrives.length; i++) {
      flags[i] = EncoderedDrives[i].getClosedLoopError() < 500;
    }
    return flags;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopDrives());
  }
}