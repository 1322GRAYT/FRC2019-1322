/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trigger;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class TurnOffAutoArm extends Trigger {
  @Override
  public boolean get() {
    return Math.abs(Robot.m_oi.AuxStick.getLeftStickY()) > 0.3 && Robot.ARM.AUTOMATIC_ACTIVE ;
  }
}
