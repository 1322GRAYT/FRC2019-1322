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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.calibrations.K_Arm;
import frc.robot.models.GamePieces;

public class CC_ArmHoldToBall extends Command {
  /**
   *
   */

  
  Button refButton;
  private boolean tFlag;
  final static Timer buttonTimer = new Timer();

  public CC_ArmHoldToBall(Button refButton) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ARM);
    this.refButton = refButton;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    buttonTimer.reset();
    buttonTimer.start();
    tFlag = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!tFlag) {
      if (buttonTimer.get() > K_Arm.BUTTON_TIMEOUT && refButton.get() || Robot.ARM.getGamePieceType() == GamePieces.HatchPanel) {
        Robot.ARM.resetToFloorCargoPickup();
        Robot.ARM.MMArm(Robot.ARM.getCurrenPositionData().location);
        tFlag = true;
      } else if (!refButton.get()) {
        Robot.ARM.incrementPosition();
        Robot.ARM.MMArm(Robot.ARM.getCurrenPositionData().location);
        tFlag = true;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.ARM.liftRawPosition() - Robot.ARM.getCurrenPositionData().location) < K_Arm.TOLERANCE) && tFlag;
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
