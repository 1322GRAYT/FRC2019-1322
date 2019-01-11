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

public class TeleopDrives extends Command{
    // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    requires(Robot.DRIVES);
    SmartDashboard.putNumberArray("Velocity", Robot.DRIVES.rawVelocities());

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.DRIVES.DriveInVoltage(Robot.m_oi.gamePad1.getY(Hand.kLeft),
      Robot.m_oi.gamePad1.getX(Hand.kLeft), 
      Robot.m_oi.gamePad1.getX(Hand.kRight));
    
      SmartDashboard.putNumberArray("Velocity", Robot.DRIVES.rawVelocities());
      SmartDashboard.putNumberArray("Velocity", Robot.DRIVES.rawPosition());


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