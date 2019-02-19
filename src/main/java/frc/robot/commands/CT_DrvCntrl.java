/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CT_DrvCntrl extends Command {

  public CT_DrvCntrl() {
    requires(Robot.DRIVES);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putNumberArray("Velocity", Robot.DRIVES.rawVelocities());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.DRIVES.DriveInVoltage(Robot.m_oi.DriverStick.getLeftStickY(), Robot.m_oi.DriverStick.getLeftStickX(),
        Robot.m_oi.DriverStick.getRightStickX());
    
    SmartDashboard.putNumber("Joystick", Robot.m_oi.DriverStick.getY(Hand.kLeft));
    SmartDashboard.putNumberArray("Velocity", Robot.DRIVES.rawVelocities());
    SmartDashboard.putNumberArray("Position", Robot.DRIVES.rawPosition());

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.DRIVES.DriveInVoltage(0, 0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}