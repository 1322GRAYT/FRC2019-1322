/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

/**
 * Class: K_Vision - Contains Control Calibrations for the Vision control system.
 */
public class K_Vision {

    /**************************************************/
	/*  Target Reference Pixel Measurements           */
	/**************************************************/	 	
	
	/** KVSN_Cnt_PxlTgtTop: Width of the top of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final int KVSN_Cnt_PxlTgtTop = 70;  // pixels

  	/** KVSN_Cnt_PxlTgtBtm: Width of the bottom of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final int KVSN_Cnt_PxlTgtBtm = 50; // pixels

	/** KVSN_Cnt_PxlTgtLt: Length of the left side of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final int KVSN_Cnt_PxlTgtLt = 40; // pixels

  	/** KVSN_Cnt_PxlTgtRt: Length of the left side of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final int KVSN_Cnt_PxlTgtRt = 40; // pixels


    /**************************************************/
	/*  Target Reference Physical Measurements        */
	/**************************************************/	 	
	
	/** KVSN_l_RefTgtTop: Width of the top of the target image in
     * measured inches. */
    public static final double KVSN_l_RefTgtTop = 14.75;  // inches

  	/** KVSN_l_RefTgtBtm: Width of the bottom of the target image in
     * measured inches. */
    public static final double KVSN_l_RefTgtBtm = 10.875; // inches

	/** KVSN_l_RefTgtLt: Length of the left side of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final double KVSN_l_RefTgtLt = 5.5; // inches

  	/** KVSN_l_RefTgtRt: Length of the left side of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final double KVSN_l_RefTgtRt = 5.5; // inches

 	/** KVSN_l_RefTgtToCamDist: Length of 
      * the left side of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final double KVSN_l_RefTgtToCamDist = 100; // inches

}
