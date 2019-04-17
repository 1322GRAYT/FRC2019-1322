/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.LEDController.colorLED;
import frc.robot.subsystems.LEDController.modeLED;


public class CC_LEDCntrl extends Command {

  private modeLED mode;
  private colorLED color;

  public CC_LEDCntrl(colorLED color) {
    requires(Robot.LEDS);
    this.color = color;
    this.mode = modeLED.Null;
  }

  public CC_LEDCntrl(colorLED color, modeLED mode) {
    requires(Robot.LEDS);
    this.color = color;
    this.mode = mode;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.LEDS.setLED_b_CmndActv(true);
    Robot.LEDS.reqLED_Color(color);
    Robot.LEDS.reqLED_Mode(mode);
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
    Robot.LEDS.setLED_b_CmndActv(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
