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
import frc.robot.subsystems.Arm;

public class CC_ArmPstnRaiseNxtHatchLvl extends Command {

  int setLevel = 0;

  public CC_ArmPstnRaiseNxtHatchLvl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ARM);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (K_Arm.MAX_PANEL_POSITION != Robot.ARM.panelPoint){
      Robot.ARM.panelPoint++;
    }
    Robot.ARM.ballPoint = 0;
    Robot.ARM.setSetPoint(Arm.getPanellevels(Robot.ARM.panelPoint));
    
    Robot.ARM.armSafety(false);
    setLevel = K_Arm.ARM_POS_DATA[Robot.ARM.getSetPoint()].location;
    Robot.ARM.MMArm(setLevel);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    toSDBoard("Arm Data", Robot.ARM.liftRawPosition(), Robot.ARM.liftRawVelocity(), setLevel, Robot.ARM.armVoltage(),
      Robot.ARM.armError());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.ARM.liftRawPosition() - setLevel) < 750;
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
