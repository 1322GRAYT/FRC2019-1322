/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Scissor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX scissorL, scissorR, scissorOut;
  private Solenoid robotLiftL, robotLiftR;
  private DigitalInput edgeSensor;

  public Scissor() {
    scissorL = new WPI_TalonSRX(RobotMap.RobotLiftAddresses[0]);
    scissorR = new WPI_TalonSRX(RobotMap.RobotLiftAddresses[1]);
    scissorR.set(ControlMode.Follower, RobotMap.RobotLiftAddresses[0]);

    edgeSensor = new DigitalInput(RobotMap.FloorSensor);

    scissorOut = new WPI_TalonSRX(RobotMap.RobotLiftExtendAddress);

    robotLiftL = new Solenoid(RobotMap.RobotLift[0]);
    robotLiftR = new Solenoid(RobotMap.RobotLift[1]);
  }

  public void liftRobot(double speed, Timer SysTmr) {
    Robot.LEDS.setMode(Robot.LEDS.ledRed, Robot.LEDS.ledModeChase);
    System.out.println("Finish SetLEDs Lift ::: " + SysTmr.get());      
    scissorL.set(ControlMode.PercentOutput, speed);
    System.out.println("Finish Set Lift :::     " + SysTmr.get());      
  }

  public void extendLift(double speed, Timer SysTmr) {
    Robot.LEDS.setMode(Robot.LEDS.ledYellow, Robot.LEDS.ledModeChase);
    System.out.println("Finish SetLEDs Drwr ::: " + SysTmr.get());      
    scissorOut.set(ControlMode.PercentOutput, speed);
    System.out.println("Finish Set Drwr :::     " + SysTmr.get());      
  }

  public boolean getFwdLimit() {
    return scissorR.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getRevLimit() {
    return scissorR.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void liftRobotPnumatic(boolean up, Timer SysTmr) {
    Robot.LEDS.setLEDs(Robot.LEDS.ledRainbow);
    System.out.println("Finish SetLEDs Pneu ::: " + SysTmr.get());      
    robotLiftL.set(up);
    System.out.println("Finish SetLft Pneu :::  " + SysTmr.get());      
    robotLiftR.set(!up);
    System.out.println("Finish SetRgt Pneu :::  " + SysTmr.get());      
  }

  public boolean getLiftRobotPnumatic() {
    return(robotLiftL.get());
  }
    

  public boolean getFloorSensor() {
    return edgeSensor.get();
  }

  public boolean getLimits(){
    return scissorL.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
