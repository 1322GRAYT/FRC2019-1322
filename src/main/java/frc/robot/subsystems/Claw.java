/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.CC_ClawAutoGrab;

/**
 * Add your docs here.
 */
public class Claw extends Subsystem {
  private Solenoid clawIn, clawOut, ejectIn, ejectOut;
  private Compressor comp;
  private DigitalInput clawDis, diskPress;

  public Claw() { //TODO: Update when Eric Pushes RobotMap
    comp = new Compressor(0);
    comp.enabled();
    clawIn = new Solenoid(RobotMap.ClawSolenoids[0]);
    clawOut = new Solenoid(RobotMap.ClawSolenoids[1]);
    ejectIn = new Solenoid(RobotMap.EjectSolenoids[0]);
    ejectOut = new Solenoid(RobotMap.EjectSolenoids[1]);
    clawDis = new DigitalInput(RobotMap.ClawSensor);
    diskPress = new DigitalInput(RobotMap.DiskSensor);
  }

  /**
   * 
   * @param eject Pushes out hatch ejectors if true
   */
  public void diskGrabber(boolean eject) {
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

  public boolean getClaw() {
    return !clawDis.get();
  }

  public boolean getDisk() {
    return diskPress.get();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CC_ClawAutoGrab());
  }
}
