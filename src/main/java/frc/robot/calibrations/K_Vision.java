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
	/*  Camera Image Resolution Settings              */
	/**************************************************/	 	
	
	/** KaVSN_Pxl_CamDim: Dimensions of the Camera Image in Pixels
    * (X-Dim,Y-Dim). */
   public static final int KaVSN_Pxl_CamDim[] = {320, 240};


  /**************************************************/
	/*  Target Reference Pixel Coordinates            */
	/**************************************************/	 	

  /** KaVSN_Pxl_RefImgCoord: Array of X,Y Coordinates of the reference
    * target camera image. 
    * Rows:    (RtUpper, RtLower, LtLower, LtUpper)
    * Columns: (X,Y) */
   public static final int KaVSN_Pxl_RefImgCoord[][] = 
     {/*  X ,  Y */ 
       { 59,  31}, /* RtUpper Corner */
       { 51,  34}, /* RtLower Corner */
       { 48,  24}, /* LtLower Corner */
       { 58,  20}  /* LtUpper Corner */
     };


  /**************************************************/
	/*  Target Reference Physical Measurements        */
	/**************************************************/	 	
	
	/** KeVSN_l_RefTgtTop: Width of the top of the target image in
   * measured inches. */
   public static final double KeVSN_l_RefTgtTop = 12.0;  // inches

  /** KeVSN_l_RefTgtBtm: Width of the bottom of the target image in
   * measured inches. */
   public static final double KeVSN_l_RefTgtBtm = 14.75; // inches

	/** KeVSN_l_RefTgtLt: Length of the equal left and right sides of
   * the target image in measured inches. */
   public static final double KeVSN_l_RefTgtSides = 5.5; // inches

 	/** KeVSN_l_RefTgtToCamDist: Length of the distance between the
   * target and the camera at the camera calibration reference
   * distance in measured inches. */
   public static final double KeVSN_l_RefTgtToCamDist = 30.75; // inches
    
}
