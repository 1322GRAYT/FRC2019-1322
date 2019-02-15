/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX Lift = new WPI_TalonSRX(RobotMap.LiftMotorAddress);
  WPI_TalonSRX BallIntake = new WPI_TalonSRX(RobotMap.BallIntakeAddress);

  public Arm() {
    Lift.configMotionCruiseVelocity(11000);
    Lift.configMotionAcceleration(12000);
    Lift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    Lift.config_kF(0, 0.11);
    Lift.config_kP(0, 0.13);
    Lift.config_kI(0, 0.0001);
    Lift.config_kD(0, 0.0);
  }

  public int liftRawPosition(){
    return Lift.getSelectedSensorPosition();
  }

  public int liftRawVelocity(){
    return Lift.getSelectedSensorVelocity();
  }

  public void intakePower(double Power){
    BallIntake.set(ControlMode.PercentOutput, Power);
  }

  public void LiftByVoltage(double Power){
    Lift.set(ControlMode.PercentOutput, Power);
  }

  public void MMArm(int Pos){
    Lift.set(ControlMode.MotionMagic, Pos);
  }

  public void VelArm(int Vel){
    Lift.set(ControlMode.Velocity, Vel);
  }

  public void armSafety(boolean safety){
    Lift.setSafetyEnabled(safety);
  }

  public double armVoltage(){
    return Lift.getMotorOutputVoltage();
  }

  public int armError(){
    return Lift.getClosedLoopError();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TC_Arm());
  }
}
