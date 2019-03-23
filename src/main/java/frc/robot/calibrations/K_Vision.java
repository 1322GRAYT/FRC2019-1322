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

     /** KaVSN_Pxl_RefImgCoordTopRt: Coordinates of the reference target
     * image camera for the Top Right image point in Pixel Coordinates
     * (X,Y). */
    public static final int KaVSN_Pxl_RefImgCoordTopRt[] =  {59, 31};

	/** KaVSN_Pxl_RefImgCoordBtmLt: Coordinates of the reference target
     * image camera for the Bottom Right image point in Pixel Coordinates
     * (X,Y). */
    public static final int KaVSN_Pxl_RefImgCoordBtmRt[] =  {51, 34};

	/** KaVSN_Pxl_RefImgCoordBtmLt: Coordinates of the reference target
     * image camera for the Bottom Left image point in Pixel Coordinates
     * (X,Y). */
    public static final int KaVSN_Pxl_RefImgCoordBtmLt[] =  {48, 24};

	/** KaVSN_Pxl_RefImgCoordTopLt: Coordinates of the reference target
     * image camera for the Top Left image point in Pixel Coordinates
     * (X,Y). */
    public static final int KaVSN_Pxl_RefImgCoordTopLt[] = {58,20};



    /**************************************************/
	/*  Target Reference Pixel Measurements           */
	/**************************************************/	 	
	
	/** KeVSN_Cnt_PxlTgtTop: Width of the top of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final int KeVSN_Cnt_PxlTgtTop = 70;  // pixels

  	/** KeVSN_Cnt_PxlTgtBtm: Width of the bottom of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final int KeVSN_Cnt_PxlTgtBtm = 50; // pixels

	/** KeVSN_Cnt_PxlTgtLt: Length of the left side of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final int KeVSN_Cnt_PxlTgtLt = 40; // pixels

  	/** KeVSN_Cnt_PxlTgtRt: Length of the left side of the target image in
     * camera pixels at the camera calibration reference distance. */
    public static final int KeVSN_Cnt_PxlTgtRt = 40; // pixels


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
