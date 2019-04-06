/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

/**
 * Class: K_Lift - Contains Control Calibrations for the Lift control system.
 */
public class K_Lift {

    /**************************************************/
	/*  Lift System Normalized Power Commands         */
	/**************************************************/	 	
	
	/** KeLFT_r_NormPwrExtdVert: Lift System Vertical Lift Normalized
     * Power Command to the Lift Motors for Extending the lift.  (0 - 1)
     */
    public static final double KeLFT_r_NormPwrExtdVert = 1;

 	/** KeLFT_r_NormPwrRtctVert: Lift System Vertical Lift Normalized
     * Power Command to the Lift Motors for Retracting the lift.  (0 - 1)
     */
    public static final double KeLFT_r_NormPwrRtctVert = 1;
   
	/** KeLFT_r_NormPwrExtdHorz: Lift System Horitontal Slide Normalized
     * Power Command to the Lift Motors for Extending the slide.  (0 - 1)
     */
    public static final double KeLFT_r_NormPwrExtdHorz = 1;

 	/** KeLFT_r_NormPwrRtctHorz: Lift System Horizontal Slide Normalized
     * Power Command to the Lift Motors for Retracting the slide.  (0 - 1)
     */
    public static final double KeLFT_r_NormPwrRtctHorz = 1;
    

}
