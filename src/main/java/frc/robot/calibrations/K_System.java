/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

/**
 * Class: K_System - Contains General System Comtrol Calibrations.
 */
public class K_System {

    /****************************************/
	/*   General System Configuration   */
	/****************************************/	 	

	/** KeSYS_b_CL_TgtEnbl: Enable Closed-Loop Vision Targeting Control. */
    public static final boolean KeSYS_b_CL_TgtEnbl = false;

	/** KeSYS_b_NewLiftEnbl: Enable the use of the new Lift Control strategy. */
    public static final boolean KeSYS_b_NewLiftEnbl = false;

	/** KeSYS_t_LoopRt: Execution Loop Rate Period for the Autonomous
	 * Command Group Controls (sec). */
	public static final float KeSYS_t_LoopRt = (float)0.020; // sec 
	 	
	/** KeSYS_b_PracticeBot: If True it indicates that the software
	 * is running on the Practice Robot which does not have the
	 * full complement of actuators and sensors so certain
	 * functionality in software should be disabled. */
    public static final boolean KeSYS_b_PracticeBot = false;	

	/** KeSYS_b_DebugEnblVsn: If True the Debug Smart Dash Display Variables
	 * Are Broadcast and Updated, if False they are turned off to maximize
	 * thru-put for controls. For the Vision System. */
    public static final boolean KeSYS_b_DebugEnblVsn = false;

	/** KeSYS_b_DebugEnblDrv: If True the Debug Smart Dash Display Variables
	 * Are Broadcast and Updated, if False they are turned off to maximize
	 * thru-put for controls.  For the Drive System.  */
    public static final boolean KeSYS_b_DebugEnblDrv = false;

	    
}
