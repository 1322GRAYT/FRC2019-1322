/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import java.util.*;
import org.opencv.core.*;
import org.opencv.calib3d.*;
import frc.robot.calibrations.K_Vision;

/**
 * Class - Vision: Contains the Processing and Methods to Calculate
 * the Distance, Angle, and Rotation of the Robot in reference to 
 * the Field Target Markings.
 */
public class Vision extends Subsystem {
  private static final int Xcell = 0; // X-Cell for array Indexing;
  private static final int Ycell = 1; // Y-Cell for array Indexing;
  
  private int VeVSN_Cnt_PxlImgTopLt[] = new int[2];
  private int VeVSN_Cnt_PxlImgTopRt[] = new int[2];
  private int VeVSN_Cnt_PxlImgBtmLt[] = new int[2];
  private int VeVSN_Cnt_PxlImgBtmRt[] = new int[2];

  private int VeVSN_Cnt_PxlWidthBtm;
  private int VeVSN_Cnt_PxlWidthTop;
  private int VeVSN_Cnt_PxlHeightLt;
  private int VeVSN_Cnt_PxlHeightRt;

  private int VeVSN_Cnt_PxlCamFocalPt = (int)0;


  /*******************************/
  /* Public Class Functions    */
  /*******************************/




  /*******************************/
  /* Internal Class Functions    */
  /*******************************/
   
  
   /**
    * Method: calcVSN_PxlDimensionsTgt - Calculate the dimensions
    * of the vision target in pixels, i.e. the length of all the sides.
    */
    public void calcVSN_GeometryTgt() {
      boolean test;
      List<Point3> objPoints = new ArrayList<Point3>();
      MatOfPoint3f objPointsMat = new MatOfPoint3f();
      List<Point> imgPoints = new ArrayList<Point>();
      MatOfPoint2f imgPointMat = new MatOfPoint2f();
      Mat rotMat;
      Mat transVec;

      /* Place Object Measurements in Object Coordinates into a single Object Measurement array */ 
      objPoints.add(new Point3(VeVSN_Cnt_PxlImgTopLt[Xcell],VeVSN_Cnt_PxlImgTopLt[Ycell],0));
      objPoints.add(new Point3(0,0,0));
      objPoints.add(new Point3(VeVSN_Cnt_PxlImgBtmRt[Xcell],VeVSN_Cnt_PxlImgBtmRt[Ycell],0));
      objPoints.add(new Point3(VeVSN_Cnt_PxlImgTopRt[Xcell],VeVSN_Cnt_PxlImgTopRt[Ycell],0));
      /* Build Object Matrix from Array */ 
      objPointsMat.fromList(objPoints);

      /* Place Image Measurements in Camera Coordinates into a single Image Measurement array */ 
      imgPoints.add(new Point(VeVSN_Cnt_PxlImgTopLt[Xcell],VeVSN_Cnt_PxlImgTopLt[Ycell]));
      imgPoints.add(new Point(VeVSN_Cnt_PxlImgBtmLt[Xcell],VeVSN_Cnt_PxlImgBtmLt[Ycell]));
      imgPoints.add(new Point(VeVSN_Cnt_PxlImgBtmRt[Xcell],VeVSN_Cnt_PxlImgBtmRt[Ycell]));
      imgPoints.add(new Point(VeVSN_Cnt_PxlImgTopRt[Xcell],VeVSN_Cnt_PxlImgTopRt[Ycell]));
      /* Build Object Matrix from Array */ 
      imgPointsMat.fromList(imgPoints);


//    test = solvePnP(objPointsMat, imgPointsMat, Mat cameraMatrix, MatOfDouble distCoeffs, rotMat, transVec);
      
      calcVSN_PxlDimensionsTgt();

      }



   /**
    * Method: calcVSN_PxlDimensionsTgt - Calculate the dimensions
    * of the vision target in pixels, i.e. the length of all the sides.
    */
    private void calcVSN_PxlDimensionsTgt() {
      VeVSN_Cnt_PxlWidthBtm = calcVSN_PxlLengthLineSeg(VeVSN_Cnt_PxlImgBtmRt, VeVSN_Cnt_PxlImgBtmLt);
      VeVSN_Cnt_PxlWidthTop = calcVSN_PxlLengthLineSeg(VeVSN_Cnt_PxlImgTopRt, VeVSN_Cnt_PxlImgTopLt);
      VeVSN_Cnt_PxlHeightLt = calcVSN_PxlLengthLineSeg(VeVSN_Cnt_PxlImgTopLt, VeVSN_Cnt_PxlImgBtmLt);
      VeVSN_Cnt_PxlHeightRt = calcVSN_PxlLengthLineSeg(VeVSN_Cnt_PxlImgTopRt, VeVSN_Cnt_PxlImgBtmRt);
      }



   /**
    * Method: calcVSN_PxlLengthLineSeg - Calculate the length of a
    * line segment given the X-Y coordinates of the two end-points
    * of the line.  Uses the Pythagreon Theorem.
    * @param1: Array of Cartesian Coordinates of Line Segment PointA (int)
    * @param2: Array of Cartesian Coordinates of Line Segment PointB (int)
    * @return: Length of Line Segment in Camera Pixels (int)

    */
    private int calcVSN_PxlLengthLineSeg(int LeVSN_Cnt_PxlPntA[], int LeVSN_Cnt_PxlPntB[]) {
      int diffX, diffY, hyp2;
      double length;
  
      /* Calculate the delta between the points in the X-dimension. */
      diffX = LeVSN_Cnt_PxlPntA[Xcell] - LeVSN_Cnt_PxlPntB[Xcell];
  
      /* Calculate the delta between the points in the Y-dimension. */
      diffY = LeVSN_Cnt_PxlPntA[Ycell] - LeVSN_Cnt_PxlPntB[Ycell];
  
      /* calculate the hypoteneuse */
      hyp2 = diffX^2 + diffY^2;
      length = Math.sqrt((double)hyp2);
  
      return ((int)length);
      }
  
  
  
    
  /*****************************************************/
  /*  Calculate Camera Focal Point                     */
  /*  F = (P x D) / W                                  */
  /*    F = Camera Focal Point (pixel)                 */
  /*    P = Reference Target Dimension (pixel)         */
  /*    W = Reference Target Dimension (inch)          */
  /*    D = Reference Camera to Target Distance (inch) */
  /*****************************************************/	 	
  /**
   * Method: calcVSN_CamFocalPt - Calculate the Camera Focal Point
   * in Pixels from the reference calibration measurments of Target
   * Width in both inches and pixels and the distance from camera and
   * target based on empirical reference measurements.
   */
  private void calcVSN_CamFocalPt() {
    double LeVSN_l_FocalPt, LeVSN_l_RefTgtBtm;

    /* To protect against a divide by zero error */
    LeVSN_l_RefTgtBtm = K_Vision.KVSN_l_RefTgtBtm;
    if (LeVSN_l_RefTgtBtm <= 1) {
      LeVSN_l_RefTgtBtm = 1;
      }
      
    LeVSN_l_FocalPt = ((double)K_Vision.KVSN_Cnt_PxlTgtBtm * K_Vision.KVSN_l_RefTgtToCamDist)/
                      (LeVSN_l_RefTgtBtm);

	  VeVSN_Cnt_PxlCamFocalPt = (int)LeVSN_l_FocalPt;          
    }


  /****************************************************/
  /*  Calculate Current Camera to Target Distance     */
  /*  D' = (W x F) / P'                               */
  /*    F = Camera Focal Point (pixel)                */
  /*    P' = Current Target Dimension (pixel)         */
  /*    W = Reference Target Dimension (inch)         */
  /*    D' = Current Camera to Target Distance (inch) */
  /****************************************************/	 	
  /**
   * Method: calcVSN_TgtDist - Calculates the current distance the
   * target is from the robot based on the camera focal length in
   * pixels, the Reference Target Dimension of the target and the
   * present reading of target width in pixels.
   */
   private void calcVSN_TgtDist(int pixels) {
     double LeVSN_l_Dist;;

     /* To protect against a divide by zero error */
     if (pixels <= 1) {
       pixels = 1;
       }
        
     LeVSN_l_Dist = ((double)K_Vision.KVSN_l_RefTgtBtm * VeVSN_Cnt_PxlCamFocalPt)/
                      (pixels);

     }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
