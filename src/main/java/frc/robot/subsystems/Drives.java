package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Drives extends Subsystem {
  TalonSRX FrontLeft, FrontRight, RearLeft, RearRight;

  public Drives(){
      FrontLeft = new TalonSRX(RobotMap.FLDrive);
      FrontRight = new TalonSRX(RobotMap.FRDrive);
      RearLeft = new TalonSRX(RobotMap.RLDrive);
      RearRight = new TalonSRX(RobotMap.RRDrive);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}