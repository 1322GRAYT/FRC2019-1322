/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

/**
 * Class: K_Tele - Contains Control Calibrations for the Tele-Operation control
 * mode.
 */
public class K_Tele {

    /**************************************************/
	/*  Lift Horizontal Normalized Power for Tele-Op  */
	/**************************************************/	 	
	
	/** KLFT_r_TelePwrHorizExtend: Normalized Power Level commanded to the motors
     * when extending the horizontal slide of the Lift Mechanism. */
    public static final double KLFT_r_TelePwrHorizExtend = 1.0;

    /** KLFT_r_TelePwrHorizRetract: Normalized Power Level commanded to the motors
     * when retracting the horizontal slide of the Lift Mechanism. */
    public static final double KLFT_r_TelePwrHorizRetract = -1.0;

    /** KLFT_r_TelePwrHorizDsbl: Normalized Power Level commanded to the motors
     * when disabling the horizontal slide control. */
    public static final double KLFT_r_TelePwrHorizDsbl = 0;



    /**************************************************/
	/*  Lift Vertical Normalized Power for Tele-Op  */
	/**************************************************/	 	
	
	/** KLFT_r_TelePwrVertRaise: Normalized Power Level commanded to the motors
     * when extending/raising the vertical jack of the Lift Mechanism. */
    public static final double KLFT_r_TelePwrVertRaise = 1.0;

    /** KLFT_r_PwrVertLowerTele: Normalized Power Level commanded to the motors
     * when retracting/lowering the vertical jack of the Lift Mechanism. */
    public static final double KLFT_r_PwrVertLowerTele = -1.0;

    /** KLFT_r_PwrVertDsblTele: Normalized Power Level commanded to the motors
     * when disabling the vertical jack control. */
    public static final double KLFT_r_PwrVertDsblTele = 0;
    
}
