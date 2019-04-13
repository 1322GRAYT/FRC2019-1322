/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;
import frc.robot.models.CustomXbox;
import frc.robot.models.EncoderConversions;
import frc.robot.subsystems.Lift.*;
import frc.robot.calibrations.K_System;


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
  
    //When A Button is pressed, retract Lift drawer slide
    DriverStick.aButton.whileHeld(new CT_LiftInpHorz(ActuatorSt.Retract));  //1
    //When B Button is pressed, extend lift drawer slide
    DriverStick.bButton.whileHeld(new CT_LiftInpHorz(ActuatorSt.Extend));   //-1

    //When Right Trigger is pressed, lift bot
    DriverStick.rightTriggerButton.whileHeld(new CT_LiftInpVert(ActuatorSt.Extend)); // 1

    //When Left Trigger is pressed, lower bot
    DriverStick.leftTriggerButton.whileHeld(new CT_LiftInpVert(ActuatorSt.Retract)); // -1

    //When Right Bumper is pressed, retract lift pnumatics    
    DriverStick.rightBumper.whileHeld(new CT_LiftInpStblzr(ActuatorSt.Retract)); // 1
    //When Left Bumper is pressed, extend lift pnumatics
    DriverStick.leftBumper.whileHeld(new CT_LiftInpStblzr(ActuatorSt.Extend)); // -1
  

    //When X Button is pressed, switch Cameras
    DriverStick.xButton.whenPressed(new CC_CamSwitch());


    /* Drive Target System Robot Tracking */ 
    DriverStick.startButton.whileHeld(new CC_AutoTgtTrkEnbl());    
    DriverStick.startButton.whenReleased(new CC_AutoTgtTrkDsbl());    

    /* Drive Target System Robot Drive Forward */ 
    DriverStick.selectButton.whileHeld(new CC_AutoTgtDrvEnbl());
    DriverStick.selectButton.whenReleased(new CC_AutoTgtDrvDsbl());


    // Arm Controls
    AuxStick.yButton.whenPressed(new CC_ArmHoldPanel(AuxStick.yButton));
    AuxStick.xButton.whenPressed(new CC_ArmHoldToBall(AuxStick.xButton));

    //When "A" Button is pressed, eject Hatch Panel, When Released, Retract pnuematics
    AuxStick.aButton.whenPressed(new CC_ClawHatchCntrl(false));
    AuxStick.bButton.whenPressed(new CC_ClawHatchCntrl(true));
    //When Right Bumper is pressed, toggle between push claw out/in; when Left bumper is pressed, pull claw in
    AuxStick.rightBumper.toggleWhenPressed(new CC_ClawBallCntrl());
    AuxStick.leftBumper.whenPressed(new CC_ClawRtct());
  }

}
