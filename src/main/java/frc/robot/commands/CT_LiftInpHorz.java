/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class CT_LiftInpHorz extends Command {
  double InpRqst;
  public CT_LiftInpHorz(double InpRqst) {
  /* InpRqst: 0 = Disabled, 1 = Extend, -1 = Retract */
  this.InpRqst = InpRqst;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  
    if (InpRqst == 1) {
      Robot.LIFT.setLFT_DrwrExtdRqst(true);
      Robot.LIFT.setLFT_DrwrRtctRqst(false);
    }
    else if (InpRqst == -1) {
      Robot.LIFT.setLFT_DrwrExtdRqst(false);
      Robot.LIFT.setLFT_DrwrRtctRqst(true);
    }
    else {
      Robot.LIFT.setLFT_DrwrExtdRqst(false);
      Robot.LIFT.setLFT_DrwrRtctRqst(false);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.LIFT.setLFT_DrwrExtdRqst(false);
    Robot.LIFT.setLFT_DrwrRtctRqst(false);
  }

}
