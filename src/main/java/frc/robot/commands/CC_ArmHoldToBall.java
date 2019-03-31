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
import frc.robot.calibrations.K_Arm;
import frc.robot.models.GamePieces;

public class CC_ArmHoldToBall extends Command {

  final static Timer buttonTimer = new Timer();
  private Button buttonRef;
  private boolean tFlag;

  public CC_ArmHoldToBall(Button buttonRef) {
    requires(Robot.ARM);
    this.buttonRef = buttonRef;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.ARM.armSafety(false);
    buttonTimer.reset();
    buttonTimer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!tFlag) {
      System.out.println("Waiting!");
      if (((buttonTimer.get() > K_Arm.BUTTON_TIMEOUT) || (Robot.ARM.getGamePieceType() != GamePieces.Cargo))
          && !tFlag) {
        Robot.ARM.resetToFloorCargoPickup();
        tFlag = true;
      } else if (!buttonRef.get() && !tFlag) {
        Robot.ARM.incrementPosition();
        tFlag = true;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return tFlag;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.ARM.armSafety(true);
    System.out.println(Robot.ARM.getGamePieceType().name() + "  " + Robot.ARM.getCurrenPositionData().name);
    System.out.println("Complete!");
    tFlag = false;
    buttonTimer.stop();
    buttonTimer.reset();
    Robot.ARM.AUTOMATIC_ACTIVE = true;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
