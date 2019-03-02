/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.calibrations.K_Arm;
import frc.robot.subsystems.Arm;

/**
 * Add your docs here.
 */
public class CC_ArmPstnRaiseNxtBallLvl_2 extends InstantCommand {
  /**
   * Add your docs here.
   */
  public CC_ArmPstnRaiseNxtBallLvl_2() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ARM);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (K_Arm.MAX_BALL_POSITION != Robot.ARM.ballPoint){
      Robot.ARM.ballPoint++;
    }
    Robot.ARM.AUTOMATIC_ACTIVE = true;
    Robot.ARM.panelPoint = 0;
    Robot.ARM.setSetPoint(Arm.getBalllevels(Robot.ARM.ballPoint));
  }

}