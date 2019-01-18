/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.models;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Add your docs here.
 */
public class XboxTrigger extends Button {
    private final Joystick joystick;
    private final int axis;

	public XboxTrigger(Joystick joystick, int axis) {
        this.joystick = joystick;
        this.axis = axis;
	}

	public boolean get() {
		return joystick.getRawAxis(axis) > .25;
	}
}
