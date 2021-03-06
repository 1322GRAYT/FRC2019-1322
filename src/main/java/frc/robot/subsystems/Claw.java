/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.calibrations.K_System;
import frc.robot.commands.CC_ClawAutoGrab;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.LEDController.clawLED;


/**
 * Add your docs here.
 */
public class Claw extends Subsystem {
  private Solenoid clawIn, clawOut, ejectIn, ejectOut;
  private Compressor comp;
  private DigitalInput clawDis, diskPress;
  WPI_TalonSRX BallIntake;
  private Relay testBotCompressor;

  public Claw() { 
    comp = new Compressor(0);
    comp.enabled();
    clawIn = new Solenoid(RobotMap.ClawSolenoids[0]);
    clawOut = new Solenoid(RobotMap.ClawSolenoids[1]);
    ejectIn = new Solenoid(RobotMap.EjectSolenoids[0]);
    ejectOut = new Solenoid(RobotMap.EjectSolenoids[1]);
    clawDis = new DigitalInput(RobotMap.ClawSensor);
    diskPress = new DigitalInput(RobotMap.DiskSensor);
    BallIntake = new WPI_TalonSRX(RobotMap.BallIntakeAddress);
    clawIn.set(false);
    clawOut.set(true);
    
    if(K_System.KeSYS_b_PracticeBot) {
      testBotCompressor = new Relay(RobotMap.PracticeBotCompressor);
    }
    
  }

  /**
   * 
   * @param eject Pushes out hatch ejectors if true
   */
  public void diskGrabber(boolean eject) {
    if(eject) {
      Robot.LEDS.setLED_ActnClaw(clawLED.PanelEjct);
    } else {
      Robot.LEDS.setLED_ActnClaw(clawLED.PanelLtch);
    }
    ejectOut.set(eject);
    ejectIn.set(!eject);
  }

/**
 * 
 * @param out Pushes out the claw if true
 */
  public void controlClaw(boolean out) {
    if(out) {
      Robot.LEDS.setLED_ActnClaw(clawLED.ClawExtd);
    } else {
      Robot.LEDS.setLED_ActnClaw(clawLED.ClawRtct);
    }
    clawIn.set(out);
    clawOut.set(!out);
  }

  public boolean getBallClawStatus(){
    return clawOut.get();
  }

  public boolean getClaw() {
    return !clawDis.get();
  }

  public boolean getDisk() {
    return diskPress.get();
  }

  public void intakePower(double Power){
    BallIntake.set(ControlMode.PercentOutput, Power);
  }

  public boolean getPressureSwitch() {
    return comp.getPressureSwitchValue();
  }

  public void controlCompressor(boolean enable) {
    testBotCompressor.set((enable) ? Relay.Value.kForward : Relay.Value.kOff);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CC_ClawAutoGrab());
  }
}
