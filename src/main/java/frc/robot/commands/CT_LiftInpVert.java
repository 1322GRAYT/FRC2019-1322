/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.subsystems.Lift.*;


public class CT_LiftInpVert extends Command {
  ActuatorSt InpRqst;
  public CT_LiftInpVert(ActuatorSt InpRqst) {
  /* InpRqst: 0 = Disabled, 1 = Extend, -1 = Retract */
  this.InpRqst = InpRqst;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  
    if (InpRqst == ActuatorSt.Extend) {
      Robot.LIFT.setLFT_b_JackExtdRqst(true);
      Robot.LIFT.setLFT_b_JackRtctRqst(false);
    }
    else if (InpRqst == ActuatorSt.Retract) {
      Robot.LIFT.setLFT_b_JackExtdRqst(false);
      Robot.LIFT.setLFT_b_JackRtctRqst(true);
    }
    else {  /* (InpRqst == ActuatorSt.Hold) */
      Robot.LIFT.setLFT_b_JackExtdRqst(false);
      Robot.LIFT.setLFT_b_JackRtctRqst(false);
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
    Robot.LIFT.setLFT_b_JackExtdRqst(false);
    Robot.LIFT.setLFT_b_JackRtctRqst(false);
  }

}
