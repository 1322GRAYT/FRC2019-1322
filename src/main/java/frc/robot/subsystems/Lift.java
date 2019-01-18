/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TC_Lift;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX[] LiftMotors = {
    new TalonSRX(RobotMap.LiftMotorAddresses[0]),
    new TalonSRX(RobotMap.LiftMotorAddresses[1])
  };

  public Lift() {
    //Set Encoder
    LiftMotors[0].configSelectedFeedbackSensor(FeedbackDevice.SensorDifference.QuadEncoder);
    //Set Inverted, because the gearbox we use requires it
    LiftMotors[1].setInverted(true);
  }


  /**
   * 
   * @param speed Speed to set the lift motors
   */
  public void setSpeed(double speed) {
    speed = Robot.deadzonify(speed);
    for(var i = 0; i < LiftMotors.length; i++){
      LiftMotors[i].set(ControlMode.PercentOutput, speed);
    }
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TC_Lift());
  }
}
