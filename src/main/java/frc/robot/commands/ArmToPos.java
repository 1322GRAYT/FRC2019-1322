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

public class ArmToPos extends Command {
  int Place = 0;

  public ArmToPos(int pos) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.PTLIFT);
    this.Place = pos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.PTLIFT.armSafety(false);
    Robot.PTLIFT.MMArm(Place);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    toSDBoard("Arm Data", Robot.PTLIFT.liftRawPosition(), Robot.PTLIFT.liftRawVelocity(), Place,
        Robot.PTLIFT.armVoltage(), Robot.PTLIFT.armError());
    System.out.println("Still Here!");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.PTLIFT.liftRawPosition() - Place) < 250;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.PTLIFT.armSafety(true);
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
