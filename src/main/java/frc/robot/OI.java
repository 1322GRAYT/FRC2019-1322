/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;
import frc.robot.models.CustomXbox;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  //Because the WPI bundled one is dumbbbbbbbbbbb
  public CustomXbox DriverStick = new CustomXbox(0);
  public CustomXbox AuxStick = new CustomXbox(1);

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public OI() {
    //When "A" Button is pressed, eject Hatch Panel, When Released, Retract pnuematics
    AuxStick.aButton.whenPressed (new CC_ClawHatchCntrl(false));
    AuxStick.bButton.whenPressed(new CC_ClawHatchCntrl(true));
    //When Right Bumper is pressed, push claw out, when Left bumper is pressed, pull claw in
    AuxStick.rightBumper.whenPressed(new CC_ClawBallCntrl(true));
    AuxStick.leftBumper.whenPressed(new CC_ClawBallCntrl(false));
    //When A Button is pressed, extend Lift
    DriverStick.aButton.whenPressed(new CC_LiftHorizSpdTgt(1));
    DriverStick.aButton.whenReleased(new CC_LiftHorizSpdTgt(0));
    //When B Button is pressed, retract lift
    DriverStick.bButton.whenPressed(new CC_LiftHorizSpdTgt(-1));
    DriverStick.bButton.whenReleased(new CC_LiftHorizSpdTgt(0));
    //When Right Trigger is pressed, lift bot
    DriverStick.rightTriggerButton.whenPressed(new CC_LiftVertSpdTgt(.5));
    DriverStick.rightTriggerButton.whenReleased(new CC_LiftVertSpdTgt(0));
    //When Left Trigger is pressed, lower bot
    DriverStick.leftTriggerButton.whenPressed(new CC_LiftVertSpdTgt(-.5));
    DriverStick.leftTriggerButton.whenReleased(new CC_LiftVertSpdTgt(0));
    //When Right Bumper is pressed, extend lift pnumatics
    DriverStick.rightBumper.whenPressed(new CC_LiftChassisWhlCntrl(true));
    //When Left Bumper is pressed, retract lift pnumatics
    DriverStick.leftBumper.whenPressed(new CC_LiftChassisWhlCntrl(false));

    AuxStick.xButton.whenPressed(new CC_ArmPstnRaiseNxtBallLvl());
    AuxStick.yButton.whenPressed(new CC_ArmPstnRaiseNxtHatchLvl());
    AuxStick.selectButton.whenPressed(new CC_ArmPstnRaiseNxtBallLvl());
    AuxStick.startButton.whenPressed(new CC_ArmPstnRaiseNxtHatchLvl());

  }

}
