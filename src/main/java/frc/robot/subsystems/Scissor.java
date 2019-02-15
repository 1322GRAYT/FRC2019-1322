/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Scissor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX scissorL, scissorR, scissorOut;
  private Solenoid robotLiftL, robotLiftR;

  public Scissor() {
    scissorL = new WPI_TalonSRX(RobotMap.RobotLiftAddresses[0]);
    scissorR = new WPI_TalonSRX(RobotMap.RobotLiftAddresses[1]);
    scissorR.set(ControlMode.Follower, RobotMap.RobotLiftAddresses[0]);

    scissorOut = new WPI_TalonSRX(RobotMap.RobotLiftExtendAddress);

    robotLiftL = new Solenoid(RobotMap.RobotLift[0]);
    robotLiftR = new Solenoid(RobotMap.RobotLift[1]);
  }

  public void liftRobot(double speed) {
    scissorL.set(ControlMode.PercentOutput, speed);
  }

  public void extendLift(double speed) {
    scissorOut.set(ControlMode.PercentOutput, speed);
  }

  public boolean getFwdLimit() {
    return scissorR.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getRevLimit() {
    return scissorR.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void liftRobotPnumatic(boolean up) {
    robotLiftL.set(up);
    robotLiftR.set(!up);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
