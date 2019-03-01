/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.calibrations.K_Arm;


/*********************************
 * THIS CODE IS TO BE IMPLEMENTED WHEN EVERYONE IS GOOD WITH IT
 */

public class CT_ArmCntrl_2 extends Command {
  public CT_ArmCntrl_2() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ARM);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (Robot.ARM.AUTOMATIC_ACTIVE){
      Robot.ARM.MMArm(K_Arm.ARM_POS_DATA[Robot.ARM.getSetPoint()].location);
    }
    Robot.ARM.armSafety(!Robot.ARM.AUTOMATIC_ACTIVE);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!Robot.ARM.AUTOMATIC_ACTIVE){
      Robot.ARM.LiftByVoltage(Robot.m_oi.AuxStick.getLeftStickY());
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
