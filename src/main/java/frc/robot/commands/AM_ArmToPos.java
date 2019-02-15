/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class AM_ArmToPos extends Command {
  int pos = 0;

  public AM_ArmToPos(int pos) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ARM);
    this.pos = pos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.ARM.armSafety(false);
    Robot.ARM.MMArm(pos);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.ARM.MMArm(pos);
    toSDBoard("Arm Data", Robot.ARM.liftRawPosition(), Robot.ARM.liftRawVelocity(), pos,
        Robot.ARM.armVoltage(), Robot.ARM.armError());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.ARM.liftRawPosition() - pos) < 1000;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.ARM.armSafety(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  public void toSDBoard(String Name, double... toSDB) {
    SmartDashboard.putNumberArray(Name, toSDB);
  }

}
