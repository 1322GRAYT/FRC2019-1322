/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.models.GamePieces;

public class CC_ArmHoldPanel extends Command {

  Timer buttonTimer = new Timer();
  private Button buttonRef;

  public CC_ArmHoldPanel(Button buttonRef) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ARM);
    this.buttonRef = buttonRef;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    buttonTimer.reset();
    buttonTimer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if ((buttonTimer.get() > 0.5) || (Robot.ARM.getGamePieceType() != GamePieces.HatchPanel)){
      Robot.ARM.resetToHABPanelPickup();
    } else if (!buttonRef.get()) {
      Robot.ARM.incrementPosition();

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
