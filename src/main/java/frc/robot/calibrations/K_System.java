/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.*;


/**
 * Class: K_System - Contains General System Comtrol Calibrations.
 */
public class K_System {


    /****************************************/
	/*   General System Configuration   */
	/****************************************/	 	

	/** KeSYS_b_CL_DrvTgtEnbl: Enable Closed-Loop Vision Targeting Drive Control. */
    public static final boolean KeSYS_b_CL_DrvTgtEnbl = true;

	/** KeSYS_b_NewLiftEnbl: Enable the use of the new Lift Control strategy. */
    public static final boolean KeSYS_b_NewLiftEnbl = true;

	/** KeSYS_t_LoopRt: Execution Loop Rate Period for the Autonomous
	 * Command Group Controls (sec). */
	public static final float KeSYS_t_LoopRt = (float)0.020; // sec 
	 	
	/** KeSYS_b_PracticeBot: If True it indicates that the software
	 * is running on the Practice Robot which does not have the
	 * full complement of actuators and sensors so certain
	 * functionality in software should be disabled. */
    public static final boolean KeSYS_b_PracticeBot = false;	

	/** KeSYS_e_DebugEnblVsn: Used to enable the broadcast, update, and display
	 * of data variables for instrumentation view, this can be via Smart
	 * Dashboard, the RoboRio Log, or Both, or Disabled completely to free
	 * up through=put for competition play.  This configuration is for the
	 * Vision System variables.  */
	public static final DebugSlct KeSYS_e_DebugEnblVsn = DebugSlct.DebugDsbl;

	/** KeSYS_e_DebugEnblCL: Used to enable the broadcast, update, and display
	 * of data variables for instrumentation view, this can be via Smart
	 * Dashboard, the RoboRio Log, or Both, or Disabled completely to free
	 * up through=put for competition play.  This configuration is for the
	 * Closed-Loop Nav System variables.  */
	public static final DebugSlct KeSYS_e_DebugEnblCL = DebugSlct.DebugDsbl;

	/** KeSYS_e_DebugEnblDrv: Used to enable the broadcast, update, and display
	 * of data variables for instrumentation view, this can be via Smart
	 * Dashboard, the RoboRio Log, or Both, or Disabled completely to free
	 * up through=put for competition play.  This configuration is for the
	 * Drive System variables.  */
	public static final DebugSlct KeSYS_e_DebugEnblDrv = DebugSlct.DebugDsbl;

	/** KeSYS_e_DebugEnblLft: Used to enable the broadcast, update, and display
	 * of data variables for instrumentation view, this can be via Smart
	 * Dashboard, the RoboRio Log, or Both, or Disabled completely to free
	 * up through=put for competition play.  This configuration is for the
	 * Lift System variables.  */
	public static final DebugSlct KeSYS_e_DebugEnblLft = DebugSlct.DebugDsbl;
   
}
