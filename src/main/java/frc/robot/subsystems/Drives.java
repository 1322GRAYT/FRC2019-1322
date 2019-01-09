package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

enum DrivePosition {FrontLeft, FrontRight, RearLeft, RearRight}

public class Drives extends Subsystem {
  TalonSRX FL1, FR1, RL1, RR1; // Setter SRXs
  TalonSRX FL2, FR2, RL2, RR2; // Follower SRXs
  

  public Drives(){
      FL1 = new TalonSRX(RobotMap.FL1Drive);
      FR1 = new TalonSRX(RobotMap.FR1Drive);
      RL1 = new TalonSRX(RobotMap.RL1Drive);
      RR1 = new TalonSRX(RobotMap.RR1Drive);

      FL2 = new TalonSRX(RobotMap.FL2Drive);
      FR2 = new TalonSRX(RobotMap.FR2Drive);
      RL2 = new TalonSRX(RobotMap.RL2Drive);
      RR2 = new TalonSRX(RobotMap.RR2Drive);

      FL2.set(ControlMode.Follower, FL1.getDeviceID());
      FR2.set(ControlMode.Follower, FR1.getDeviceID());
      RL2.set(ControlMode.Follower, RL1.getDeviceID());
      RR2.set(ControlMode.Follower, RR1.getDeviceID());
  }

  /************
   * Unit Conversions
   * 
   * inToTicks converts linear inches to encoder ticks
   *    For use to read encoder counts to 
   * ticksToIn converts encoder ticks to linear inches
   */
  private final int countsPerCycle = 200 * 4; // 200 ticks at 4x encoder (4x Encoder counts Up and Down)
  private final double diaOfWheel = 4; // in inches
  private final double circOfWheel = Math.PI * diaOfWheel; // D * PI
  private final double gearRatio = (54/28)*(22/12); // 28 to 54 to 12 to 22
  public int inToTicks(double Inches){
    return (int)(Inches * (gearRatio * countsPerCycle / circOfWheel));
  }

  public double ticksToIn(int Ticks){
    return ((double)Ticks) / (gearRatio * countsPerCycle / circOfWheel);
  }

  /***************
   * Get raw Velocities
   * @return In Ticks per 10ms
   */
  public int[] rawVelocities(){
    var Velocities = new int[4];
    
    Velocities[0] = FL1.getSelectedSensorVelocity();
    Velocities[1] = FR1.getSelectedSensorVelocity();
    Velocities[2] = RL1.getSelectedSensorVelocity();
    Velocities[3] = FL1.getSelectedSensorVelocity();

    return Velocities;
  }


  /**********************
   * Get Raw Positions
   * @return In Ticks
   */
  public int[] rawPosition(){
    var Positions = new int[4];

    Positions[0] = FL1.getSelectedSensorPosition();
    Positions[1] = FR1.getSelectedSensorPosition();
    Positions[2] = RL1.getSelectedSensorPosition();
    Positions[3] = RR1.getSelectedSensorPosition();

    return Positions;
  }

  /*************************************
   * Drive Functions
   */
  public void DriveInVoltage(double F, double L, double R){
    var x = deadzone(L);
    var y = deadzone(F);
    var r = deadzone(R);

    double wheelSpeeds[] = {y + x + r, y - x - r, y - x + r, y + x - r};
    wheelSpeeds = normalize(scale(wheelSpeeds, 1.0));

    FL1.set(ControlMode.PercentOutput, wheelSpeeds[0]);
    FR1.set(ControlMode.PercentOutput, wheelSpeeds[1]);
    RL1.set(ControlMode.PercentOutput, wheelSpeeds[2]);
    RR1.set(ControlMode.PercentOutput, wheelSpeeds[3]);
  }

  private static double[] scale(double wheelSpeeds[], double scaleFactor) {
    for (double x :
            wheelSpeeds) {
        x *= scaleFactor;
    }
    return wheelSpeeds;
  }

private static double[] normalize(double wheelSpeeds[]) {
    double maxMagnitude = wheelSpeeds[0];
    for (double p :
            wheelSpeeds) {
        maxMagnitude = Math.max(maxMagnitude, p);
    }

    if (maxMagnitude > 1.0) {
        for (double q :
                wheelSpeeds) {
            q /= maxMagnitude;
        }
    }
    return wheelSpeeds;
  }

  private double deadzone(double power) {
    return Math.abs(power) > 0.2 ? power : 0.0 ;
  }

  /****************************
   * Autonomous Mode Functions
   */

   public void MMControl(int Distance){

    FL1.set(ControlMode.MotionMagic, Distance);
    FR1.set(ControlMode.MotionMagic, Distance);
    RL1.set(ControlMode.MotionMagic, Distance);
    RR1.set(ControlMode.MotionMagic, Distance);

   }
   public void MMControl(double Distance) {
    FL1.set(ControlMode.MotionMagic, inToTicks(Distance));
    FR1.set(ControlMode.MotionMagic, inToTicks(Distance));
    RL1.set(ControlMode.MotionMagic, inToTicks(Distance));
    RR1.set(ControlMode.MotionMagic, inToTicks(Distance));
   }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}