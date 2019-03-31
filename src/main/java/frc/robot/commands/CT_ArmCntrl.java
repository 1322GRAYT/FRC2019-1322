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
import frc.robot.calibrations.K_Arm;

public class CT_ArmCntrl extends Command {

  public CT_ArmCntrl() {
    requires(Robot.ARM);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.ARM.placeLocationTexttoSDB();
    if (Robot.ARM.AUTOMATIC_ACTIVE) {
      Robot.ARM.armSafety(false);
      Robot.ARM.MMArm(Robot.ARM.getCurrenPositionData().location);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (JoystickOveride() || withinTol()) {
      Robot.ARM.AUTOMATIC_ACTIVE = false;
      Robot.ARM.armSafety(true);
    }

    if (!Robot.ARM.AUTOMATIC_ACTIVE) {
      Robot.ARM.LiftByVoltage(Robot.m_oi.AuxStick.getLeftStickY());
    }

    Robot.ARM.placeArmDatatoSDB();
  }

  private boolean withinTol() {
    return Math.abs(Robot.ARM.liftRawPosition() - Robot.ARM.getCurrenPositionData().location) < K_Arm.TOLERANCE;
  }

  private boolean JoystickOveride() {
    return Math.abs(Robot.m_oi.AuxStick.getLeftStickY()) - 0.3 > 0;
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

  public void toSDBoard(String Name, double... toSDB) {
    SmartDashboard.putNumberArray(Name, toSDB);
  }
}
