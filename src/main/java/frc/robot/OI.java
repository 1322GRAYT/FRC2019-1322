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
    //When "A" button is pressed, eject Hatch Panel, When Released, Retract pnuematics
    AuxStick.aButton.whenPressed (new BM_ControlEject(false));
    AuxStick.aButton.whenReleased(new BM_ControlEject(true));
    //When Right Bumper is pressed, push claw out, when Left bumper is pressed, pull claw in
    AuxStick.rightBumper.whenPressed(new BM_ControlClaw(true));
    AuxStick.leftBumper.whenPressed(new BM_ControlClaw(false));
    //When A button is pressed, Extend Lift
    DriverStick.aButton.whenPressed(new BM_ExtendLift(1));
    DriverStick.aButton.whenReleased(new BM_ExtendLift(0));
    //When B button is pressed, retract lift
    DriverStick.bButton.whenPressed(new BM_ExtendLift(-1));
    DriverStick.bButton.whenReleased(new BM_ExtendLift(0));
    //When Right Trigger is pressed, lift bot
    DriverStick.rightTriggerButton.whenPressed(new BM_ScissorLift(.5));
    DriverStick.rightTriggerButton.whenReleased(new BM_ScissorLift(0));
    //When Left Trigger is pressed, lift bot
    DriverStick.leftTriggerButton.whenPressed(new BM_ScissorLift(-.5));
    DriverStick.leftTriggerButton.whenReleased(new BM_ScissorLift(0));
    //When Right Bumper is pressed, extend lift pnumatics
    DriverStick.rightBumper.whenPressed(new BM_LiftRobotPnuematic(true));
    //When Left Bimper is pressed, retract lift pnumatics
    DriverStick.leftBumper.whenPressed(new BM_LiftRobotPnuematic(false));

  }

}
