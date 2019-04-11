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
	/*  Camera System Settings              */
	/**************************************************/	 	

  /** KeVSN_Cnt_CamTgtCornMin: Minimum number of target corners
    * required to be detected before the target image is considered a
      valid image. */
   public static final int KeVSN_Cnt_CamTgtCornMin = 4;
	
	/** KaVSN_Pxl_Ca0mDim: Dimensions of the Camera Image in Pixels
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
       { 176,  96}, /* RtUpper Corner */
       { 193, 118}, /* RtLower Corner */
       { 100, 149}, /* LtLower Corner */
       { 107, 118}  /* LtUpper Corner */
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
   public static final double KeVSN_l_RefTgtToCamDist = 38.5; // inches

 	/** KeVSN_l_RbtFrameAllwnc: Robot Frame Buffer Allowance to subtract
   * off of Camera to Target Distance when determining where to place
   * target to drive robot in front of target. */
  public static final double KeVSN_l_RbtFrameAllwnc = 10.0; // inches



  /**************************************************/
	/*  Vision Target Closed-Loop Error Calibrations  */
	/**************************************************/	 	
	
	/** KeVSN_Deg_ErrDB: Closed-Loop Error Dead-Band for the
   * Vision Target X-Axis Error between the Calibrated Zero-Position
   * Cross-Hair of the target center of mass and the current position
   * Cross-Hair target center of mass. */
  public static final double KeVSN_Deg_ErrDB = 2.0;  // degrees

	
	/** KeVSN_Deg_AccumDsblErrMin: Closed-Loop Error at which it is too large
   * to consider addjusting with Integral Correction so that it is not applied
   * to the error accumulator (to prevent unnecessary accumulator wind up before
   * Feed-Forward and Proportional control have a change to make a correction). */
  public static final double KeVSN_Deg_AccumDsblErrMin = 20.0;  // degrees



}
