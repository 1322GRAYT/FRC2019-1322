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
	
	/** KSYS_b_DebugEnbl: If True the Debug Smart Dash Display Variables
	 * Are Broadcast and Updated, if False they are turned off to maximize
	 * thru-put for controls. */
    public static final boolean KCMD_b_DebugEnbl = false;
	 
	/** KCMD_t_LoopRt: Execution Loop Rate Period for the Autonomous
	 * Command Group Controls (sec). */
	 public static final float KCMD_t_LoopRt = (float)0.020; // sec 
	 	
	    
}
