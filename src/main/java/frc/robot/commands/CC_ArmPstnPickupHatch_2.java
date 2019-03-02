/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

/**
 * Add your docs here.
 */
public class CC_ArmPstnPickupHatch_2 extends InstantCommand {
  /**
   * Add your docs here.
   */
  public CC_ArmPstnPickupHatch_2() {
    super();
    requires(Robot.ARM);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.ARM.panelPoint = 0;

    Robot.ARM.AUTOMATIC_ACTIVE = true;
    Robot.ARM.ballPoint = 0;
    Robot.ARM.setSetPoint(Arm.getPanellevels(Robot.ARM.panelPoint));
  }

}
