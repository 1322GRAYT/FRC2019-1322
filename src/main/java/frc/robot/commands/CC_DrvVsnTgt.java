/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CLpid;

public class CC_DrvVsnTgt extends Command {
  public CC_DrvVsnTgt() {
    requires(Robot.PID);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   Robot.PID.setPID_Deg_PstnTgt(true, 0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.PID.managePIDRotate(); 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.PID.setPID_Deg_PstnTgt(false, 0.0);
    Robot.PID.mngPID_InitCntrl();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.PID.setPID_Deg_PstnTgt(false, 0.0);
    Robot.PID.mngPID_InitCntrl();
  }
}
