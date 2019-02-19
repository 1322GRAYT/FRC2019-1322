/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CC_LEDCntrl extends Command {

  private int mode;
  private int color;

  public CC_LEDCntrl(int color) {
    requires(Robot.LEDS);
    this.color = color;
    this.mode = -1;
  }

  public CC_LEDCntrl(int color, int mode) {
    requires(Robot.LEDS);
    this.color = color;
    this.mode = mode;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(mode > -1){
      Robot.LEDS.setMode(color, mode);
    } else {
      Robot.LEDS.setLEDs(color);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
