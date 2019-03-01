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
import frc.robot.calibrations.K_Arm;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX Lift = new WPI_TalonSRX(RobotMap.LiftMotorAddress);
  /************************************************
   * Section depreciated of ARMLEVELS, use K_Arm.ARM_POS_LEVEL to replace this variable
   * Reason for this is to provide a better model for how 
  */
  private int setPoint = 0;
  public int ballPoint = 0;
  public int panelPoint = 0;
  public boolean AUTOMATIC_ACTIVE = false;
  

  public Arm() {
    Lift.configMotionCruiseVelocity(11000);
    Lift.configMotionAcceleration(12000);
    Lift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    Lift.config_kF(0, 0.11);
    Lift.config_kP(0, 0.13);
    Lift.config_kI(0, 0.0001);
    Lift.config_kD(0, 0.0);
    Lift.configForwardSoftLimitThreshold(K_Arm.ARM_POS_DATA[K_Arm.MAX_ARM_POSITION].location); //ARMLEVELS[ARMLEVELS.length-1]);
    Lift.configForwardSoftLimitEnable(true);
  }

  /**
   * @return the balllevels
   */
  public static int getBalllevels(int level) {
    return K_Arm.BALL_POSITIONS[level];
  }

  /**
   * @return the panellevels
   */
  public static int getPanellevels(int level) {
    return K_Arm.PANEL_POSITIONS[level];
  }

  /**
   * @return the setPoint
   */
  public int getSetPoint() {
    return setPoint;
  }

  /**
   * @param setPoint the setPoint to set
   */
  public void setSetPoint(int setPoint) {
    this.setPoint = setPoint;
  }

  public int liftRawPosition() {
    return Lift.getSelectedSensorPosition();
  }

  public int liftRawVelocity(){
    return Lift.getSelectedSensorVelocity();
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
    setDefaultCommand(new CT_ArmCntrl());
  }
}
