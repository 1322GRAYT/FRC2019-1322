/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Claw extends Subsystem {
  private Solenoid clawIn, clawOut, ejectIn, ejectOut;

  public Claw() { //TODO: Update when Eric Pushes RobotMap
    clawIn = new Solenoid(RobotMap.ClawSolenoids[0]);
    clawOut = new Solenoid(RobotMap.ClawSolenoids[1]);
    ejectIn = new Solenoid(RobotMap.EjectSolenoids[0]);
    ejectOut = new Solenoid(RobotMap.EjectSolenoids[1]);
  }

  /**
   * 
   * @param eject Pushes out hatch ejectors if true
   */
  public void controlEject(boolean eject) {
    ejectOut.set(eject);
    ejectIn.set(!eject);
  }

/**
 * 
 * @param out Pushes out the claw if true
 */
  public void controlClaw(boolean out) {
    clawIn.set(out);
    clawOut.set(!out);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
