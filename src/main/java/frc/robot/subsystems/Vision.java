/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import java.util.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.opencv.core.*;
import org.opencv.calib3d.*;
import frc.robot.calibrations.K_Vision;

/**
 * Class - Vision: Contains the Processing and Methods to Calculate
 * the Distance, Angle, and Rotation of the Robot in reference to 
 * the Field Target Markings.
 */
public class Vision extends Subsystem {

  NetworkTableInstance NetTbl;
  NetworkTable LimeLightTbl;
  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry ts;
  NetworkTableEntry tshort;
  NetworkTableEntry tlong;
  NetworkTableEntry thor;
  NetworkTableEntry tvert;
  NetworkTableEntry tcornx;
  NetworkTableEntry tcorny;

  private double LL_TgtVld;        // Valid Target Acquired
  private double LL_TgtAngX;       // Horizontal Offset from CrossHair to Target (-27 to 27 Degrees)
  private double LL_TgtAngY;       // Vertical Offset from CrossHair to Target (-20.5 degrees to 20.5 degrees)
  private double LL_TgtArea;       // Target Area (0% of image to 100% of image)
  private double LL_TgtSkew;       // Target Skew or Rotation
  private double LL_TgtSideShort;  // Sidelength of shortest side of the fitted bounding box (pixels)  
  private double LL_TgtSideLong;   // Sidelength of longest side of the fitted bounding box (pixels)
  private double LL_TgtLngthHort;  // Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  private double LL_TgtLngthVert;  // Vertical sidelength of the rough bounding box (0 - 320 pixels)
  private double LL_TgtCornX[] = new double[4];  // Target Corner Coord-X: RH, RL, LL, LH
  private double LL_TgtCornY[] = new double[4];  // Target Corner Coord-Y: RH, RL, LL, LH


  private double VeVSN_Pxl_ImgWidthBtm;
  private double VeVSN_Pxl_ImgWidthTop;
  private double VeVSN_Pxl_ImgHeightLt;
  private double VeVSN_Pxl_ImgHeightRt;

  private int VeVSN_Pxl_CamFocalPt;
  private double VeVSN_l_Cam2Tgt2ndry;
  private double VeVSN_l_Cam2Tgt;
  private double VeVSN_l_Rbt2Tgt;
  private double VeVSN_deg_Rbt2Tgt;
  private double VeVSN_Deg_RbtRot;


  /* Array Indexes for building Matricies for Vision Pose Calculations */
  private static final int Xcell  = 0; // X-Cell for array Indexing;
  private static final int Ycell  = 1; // Y-Cell for array Indexing;
  private static final int Zcell  = 2; // Z-Cell for array Indexing;

  private static final int RtUpr  = 0;  // Top Right Cell for array Indexing;
  private static final int RtLwr  = 1;  // Bottom Right Cell for array Indexing;
  private static final int LtLwr  = 2;  // Bottom Left Cell for array Indexing;
  private static final int LtUpr  = 3;  // Top Left Cell for array Indexing;
  private static final int NumPts = 4;  // Total Number of Data Points;

  /**********************************************/
  /* Matricies for Object Dimensions and Images */
  /**********************************************/

  /* Matricies of Camera and Image Data */
  private MatOfPoint3f VmVSN_l_RefObj   = new MatOfPoint3f();
  private MatOfPoint2f VmVSN_Pxl_RefImg = new MatOfPoint2f();
  private Mat VmVSN_Pxl_Cam     = new Mat();
  private MatOfDouble VmVSN_k_DistCoeff = new MatOfDouble();
  private Mat VmVSM_k_RotVect   = new Mat(3,1,CvType.CV_64F);
  private Mat VmVSM_k_TransVect = new Mat(3,1,CvType.CV_64F);
  private Mat VmVSM_k_Rot       = new Mat(3,3,CvType.CV_64F);
  private Mat VmVSM_k_ImgPlaneZeroWorld = new Mat(3,3,CvType.CV_64F);

  /* Arrays of the Matrix Data for viewing via Instrumentation */
  private int VaVSN_Pxl_RefImgCoord[][] = new int[4][2];
  private double VaVSN_l_RefObjCoord[][] = new double[4][3];
  private int VaVSN_Pxl_CamMatrix[][] = new int[3][3];
  private int VaVSN_Pxl_CamImgCoord[][] = new int[4][2];



  /*******************************/
  /* Public Class Interfaces     */
  /*******************************/

 /**
   * Method: GetVSN_b_LL_TgtVld - Indication that Valid Target data has
   * been acquired.
   * @return: double - boolean (valid = true)
   */
  public double GetVSN_b_LL_TgtVld() {
    return(LL_TgtVld);
  }

 /**
   * Method: GetVSN_deg_LL_TgtAngX - Horizontal Offset from CrossHair to
   * Target (-27 to 27 degrees).
   * @return: double - degrees
   */
  public double GetVSN_deg_LL_TgtAngX() {
    return(LL_TgtAngX);
  }

 /**
   * Method: GetVSN_deg_LL_TgtAngY - Vertical Offset from CrossHair to
   * Target (-20.5 to 20.5 degrees).
   * @return: double - degrees
   */
  public double GetVSN_deg_LL_TgtAngY() {
    return(LL_TgtAngY);
  }

 /**
   * Method: GetVSN_Pct_LL_TgtArea - Target Area (0% of image to 100% of image).
   * @return: double - percent
   */
  public double GetVSN_Pct_LL_TgtArea() {
    return(LL_TgtArea);
  }

 /**
   * Method: GetVSN_Deg_LL_TgtSkew - Target Skew or Rotation
   * @return: double - degrees
   */
  public double GetVSN_Deg_LL_TgtSkew() {
    return(LL_TgtSkew);
  }

 /**
   * Method: GetVSN_Pxl_LL_TgtSideShort - Sidelength of shortest side
   * of the fitted bounding box (pixels).  
   * @return: double - pixels
   */
public double GetVSN_Pxl_LL_TgtSideShort() {
    return(LL_TgtSideShort);
  }

 /**
   * Method: GetVSN_Pxl_LL_TgtSideLong - Sidelength of longest side
   * of the fitted bounding box (pixels).  
   * @return: double - pixels
   */
  public double GetVSN_Pxl_LL_TgtSideLong() {
    return(LL_TgtSideLong);
  }

 /**
   * Method: GetVSN_Pxl_LL_TgtLngthHort - Horizontal sidelength of
   * the rough bounding box (0 - 320 pixels).
   * @return: double - pixels
   */
  public double GetVSN_Pxl_LL_TgtLngthHort() {
    return(LL_TgtLngthHort);
  }

 /**
   * Method: GetVSN_Pxl_LL_TgtLngthVert - Vertical sidelength of
   * the rough bounding box (0 - 320 pixels).  
   * @return: double - pixels
   */
  public double GetVSN_Pxl_LL_TgtLngthVert() {
    return(LL_TgtLngthVert);
  }

 /**
   * Method: GetVSN_Pxl_LL_TgtCornX - Target Corner X-Coordinates
   * RH, RL, LL, LH (0 - 320 pixels).
   * @param1: int - Xcell index (RH, RL, LL, LH)
   * @return: double - pixels X Coordinates (RH, RL, LL, LH)
   */
  public double GetVSN_Pxl_LL_TgtCornX(int LeVSN_i_CellIdx) {
    return(LL_TgtCornX[LeVSN_i_CellIdx]);
  }

// Target Corner Coord-Y: RH, RL, LL, LH (0 - 320 pixels)
 /**
   * Method: GetVSN_Pxl_LL_TgtCornY - Target Corner Y-Coordinates
   * RH, RL, LL, LH (0 - 320 pixels).
   * @param1: int - Ycell index (RH, RL, LL, LH)
   * @return: double - pixels Y Coordinates (RH, RL, LL, LH)
   */
  public double GetVSN_Pxl_LL_TgtCornY(int LeVSN_i_CellIdx) {
    return(LL_TgtCornY[LeVSN_i_CellIdx]);
  }

   /**
   * Method: GetVSN_Pxl_ImgWidthBtm - Target image width of
   * bottom horizontal of target rectangle in pixels.  
   * @return: double - pixels
   */
  public double GetVSN_Pxl_ImgWidthBtm() {
     return(VeVSN_Pxl_ImgWidthBtm); 
  }

   /**
   * Method: GetVSN_Pxl_ImgWidthTop - Target image width of
   * top horizontal of target rectangle in pixels.  
   * @return: double - pixels
   */
  public double GetVSN_Pxl_ImgWidthTop() {
    return(VeVSN_Pxl_ImgWidthTop); 
  }

   /**
   * Method: GetVSN_Pxl_ImgHeightLt - Target image height of
   * left vertical of target rectangle in pixels.  
   * @return: double - pixels
   */
  public double GetVSN_Pxl_ImgHeightLt() {
    return(VeVSN_Pxl_ImgHeightLt); 
  }

   /**
   * Method: GetVSN_Pxl_ImgHeightRt - Target image height of
   * right vertical of target rectangle in pixels.  
   * @return: double - pixels
   */
  public double GetVSN_Pxl_ImgHeightRt() {
    return(VeVSN_Pxl_ImgHeightRt); 
  }

   /**
   * Method: GetVSN_Pxl_CamFocalPt - Camera Focal Point/Length
   * in Pixels (from Camera Calibration Measurements).  
   * @return: double - pixels
   */
  public int GetVSN_Pxl_CamFocalPt() {
    return(VeVSN_Pxl_CamFocalPt); 
  }

  /**
   * Method: GetVSN_l_Cam2Tgt2ndry - Calculated Camera to Target
   * Distance (Secondary Calculation).  
   * @return: double - pixels
   */
  public double GetVSN_l_Cam2Tgt2ndry() {
    return(VeVSN_l_Cam2Tgt2ndry); 
  }

  /**
   * Method: GetVSN_l_Cam2Tgt - Calculated Camera to Target
   * Distance (Primary Calculation).  
   * @return: double - pixels
   */
  public double GetVSN_l_Cam2Tgt() {
    return(VeVSN_l_Cam2Tgt); 
  }

  /**
   * Method: VeVSN_l_Rbt2Tgt - Calculated Robot to Target Distance.  
   * @return: double - inches
   */
  public double GetVSN_l_Rbt2Tgt() {
    return(VeVSN_l_Rbt2Tgt); 
  }

  /**
   * Method: VeVSN_deg_Rbt2Tgt - Calculated Angle between the line
   * perpendicular from the target center and the line from the target
   * center to the robot.
   * @return: double - degrees
   */
  public double GetVSN_deg_Rbt2Tgt() {
    return(VeVSN_deg_Rbt2Tgt); 
  }

  /**
   * Method: GetVSN_Deg_RbtRot - Calculated Angle of Rotation of the
   * Robot wrt to the line perpendicular to the camera lens and the and
   * the line between the robot and center of target.
   * @return: double - degrees
   * */
  public double GetVSN_Deg_RbtRot() {
    return(VeVSN_Deg_RbtRot); 
  }



  /*******************************/
  /* Public Class Methods        */
  /*******************************/

   /**
    * Method: MngVSN_InitLimeLightNetTbl - Processes the Initialization
    * of the Network Tables Address Mapping for the Lime Light Camera.
    */
    public void MngVSN_InitLimeLightNetTbl() {
      NetTbl =       NetworkTableInstance.getDefault();
      LimeLightTbl = NetworkTableInstance.getDefault().getTable("limelight");
      tv = LimeLightTbl.getEntry("tv");
      tx = LimeLightTbl.getEntry("tx");
      ty = LimeLightTbl.getEntry("ty");
      ta = LimeLightTbl.getEntry("ta");
      ts = LimeLightTbl.getEntry("ts");
      tshort = LimeLightTbl.getEntry("tshort");
      tlong  = LimeLightTbl.getEntry("tlong");
      thor   = LimeLightTbl.getEntry("thor");
      tvert  = LimeLightTbl.getEntry("tvert");
      tcornx = LimeLightTbl.getEntry("tcornx");
      tcorny = LimeLightTbl.getEntry("tcorny");
    }  


 /**
    * Method: captureVSN_CamImgData - Capture the Raw Image Data from
    * the Camera.  Receives the data from the
    * Network Tables that have been transmitted from the Rpi Controller.
    */
    public boolean captureVSN_CamImgData() {
      boolean LeVSN_b_TgtAcqVld = false;

      //read values periodically
      LL_TgtVld  = tv.getDouble(0.0);
      if (LL_TgtVld == 1.0) {
        LL_TgtAngX = tx.getDouble(0.0);
        LL_TgtAngY = ty.getDouble(0.0);
        LL_TgtArea = ta.getDouble(0.0);
        LL_TgtSkew = ts.getDouble(0.0);
        LL_TgtSideShort = tshort.getDouble(0.0);
        LL_TgtSideLong  = tlong.getDouble(0.0);
        LL_TgtLngthHort = thor.getDouble(0.0);
        LL_TgtLngthVert = tvert.getDouble(0.0);
        LL_TgtCornX   = tcornx.getDoubleArray(new double[0]);
        LL_TgtCornY   = tcorny.getDoubleArray(new double[0]);    
      }

      if ((LL_TgtVld == 1.0) &&
          (LL_TgtCornX.length == 4) &&
          (LL_TgtCornY.length == 4)) {
        LeVSN_b_TgtAcqVld = true;            
      }
      return(LeVSN_b_TgtAcqVld);
    }


   /**
    * Method: MngVSN_InitCamCalibr - Processes the Camera
    * Calibration Calculations from the Camera Reference
    * Calibration Data.  Creates the Object Geometry Matrix
    * and the Reference Image Matrix, and calculates the
    * Camera Focal Length, etc..  Done during Robot Initialization
    * at the beginning of the Match.
    */
    public void MngVSN_InitCamCalibr() {
      calcVSN_RefTgtObjMat();
      calcVSN_RefTgtImgMat();
      calcVSN_CamFocalPt();
      calcVSN_CamMat();
      VmVSN_k_DistCoeff.zeros(4,1,CvType.CV_64F);
      RstVSN_CamMats();
    }

  
   /**
    * Method: RstVSN_CamMats - Resets the persistent Camera Matricies
      by loading them as zero matricies..
    */
    public void RstVSN_CamMats() {
      VmVSM_k_RotVect.zeros(3,1,CvType.CV_64F);
      VmVSM_k_TransVect.zeros(3,1,CvType.CV_64F);
      VmVSM_k_Rot.zeros(3,3,CvType.CV_64F);
      VmVSM_k_ImgPlaneZeroWorld.zeros(3,3,CvType.CV_64F);
    }


   /**
    * Method: MngVSN_CamImgProc - Calculate the dimensions
    * of the vision target in pixels, i.e. the length of all the sides.
    */
    public void MngVSN_CamImgProc() {
      parseVSN_CamImgData();
      calcVSN_CamTgtImgGeometry();
      calcVSN_TgtDist();
      calcVSN_TgtData();
    }



  /*******************************/
  /* Internal Class Methods      */
  /*******************************/   

   /**
    * Method: parseVSN_CamImgData - Update the Raw Image Data from
    * the Camera and loading it into the proper arrays for Target
    * Distance and Angle processing.
    */
    private void parseVSN_CamImgData() {
      int i;

      System.out.println("LL_TgtCornX size : " + LL_TgtCornX.length);
      System.out.println("LL_TgtCornY size : " + LL_TgtCornY.length);

      for (i=0;i<NumPts;i++) {
        VaVSN_Pxl_CamImgCoord[i][Xcell] = (int)LL_TgtCornX[i];
        VaVSN_Pxl_CamImgCoord[i][Ycell] = (int)LL_TgtCornY[i];
      }
    }


   /**
    * Method: calcVSN_TgtData - Calculate the target image pose
    * data, i.e. Target to Camera distance, Camera to Target Angle,
    * and Robot Rotation Angle wrt the Target.
    *  @return:  Indication that the PnP Calculation is has failed.
    */
    private boolean calcVSN_TgtData() {
      boolean Err_PnP;
      double x[], z[];
      Mat LmVSM_k_RotInv       = new Mat(3,3,CvType.CV_64F);
      Mat LmVSM_k_TransVectNeg = new Mat(1,3,CvType.CV_64F);
      Mat LmVSM_k_ZerosVect    = new Mat(3,1,CvType.CV_64F);

      System.out.println("RefObj Matrix : " + VmVSN_l_RefObj);
      System.out.println("RefImg Matrix : " + VmVSN_Pxl_RefImg);
      System.out.println("Camera Matrix : " + VmVSN_Pxl_Cam);

      /* Calculate the Rotation Matrix and Translation Vector */
      Err_PnP = Calib3d.solvePnP(VmVSN_l_RefObj, VmVSN_Pxl_RefImg, VmVSN_Pxl_Cam,
                                 VmVSN_k_DistCoeff, VmVSM_k_RotVect, VmVSM_k_TransVect);
  
      if (Err_PnP == false) {
        /* Calculate Distance between Target and Camera/Robot */
        x = VmVSM_k_TransVect.get(Xcell,0);
        z = VmVSM_k_TransVect.get(Zcell,0);
        VeVSN_l_Cam2Tgt = Math.sqrt(Math.pow(x[0],2) + Math.pow(z[0],2));
        VeVSN_l_Rbt2Tgt = VeVSN_l_Cam2Tgt - 0; // todo: Subtract Robot Front to Cam Distance


        /* Angle1: Calculate horiz angle from robot/camera forward and the robot-target line */
        VeVSN_deg_Rbt2Tgt = Math.atan2(x[0], z[0]);

        
        /* Initialize Local Matricies for prior to Angle2 calculation */
        LmVSM_k_RotInv.zeros(3,3,CvType.CV_64F);
        LmVSM_k_TransVectNeg.zeros(1,3,CvType.CV_64F);
        LmVSM_k_ZerosVect.zeros(3,1,CvType.CV_64F);

        /* Angle2: Calculate horiz angle between the target perpendicular and the robot-target line */
        Calib3d.Rodrigues(VmVSM_k_RotVect, VmVSM_k_Rot);
        Core.transpose(VmVSM_k_Rot,LmVSM_k_RotInv);
        Core.scaleAdd(VmVSM_k_TransVect, -1.0, LmVSM_k_ZerosVect, LmVSM_k_TransVectNeg);
        Core.multiply(LmVSM_k_RotInv, LmVSM_k_TransVectNeg, VmVSM_k_ImgPlaneZeroWorld);
        x = VmVSM_k_ImgPlaneZeroWorld.get(0,0);
        z = VmVSM_k_ImgPlaneZeroWorld.get(2,0);
        VeVSN_Deg_RbtRot = (double)Math.atan2(x[0], z[0]);
      }
      else /* (Err_PnP == true) */ {
        /* Failed to calculated proper PnP values - Clear out Peristant Matrices and Variables */
        RstVSN_CamMats();
      }

      /* free-up memory from the locally matricies */
      LmVSM_k_RotInv.release();
      LmVSM_k_TransVectNeg.release();
      LmVSM_k_ZerosVect.release();

      return(Err_PnP);
    }


   /**
    * Method: calcVSN_RefTgtObjMat - Calculate the coordinates of
    * the target object in object space in physical units and build
    * the Reference Target Object Matrix.
    */
    private void calcVSN_RefTgtObjMat() {
      double triBase, triHeight;
      int i;
     
      /* Calculate the Equal-Angular Rhombas End-Triangle Base Lengths */ 
      triBase = (Math.abs(K_Vision.KeVSN_l_RefTgtBtm - K_Vision.KeVSN_l_RefTgtTop))/2;
  
      /* Calculate the Rhombas Height via calculating the Height of the Eng-Triangles. */
      triHeight = Math.sqrt(Math.pow(K_Vision.KeVSN_l_RefTgtSides,2) - Math.pow(triBase,2));

      /*  Determine Point Coordinates: RtUpr Point */
      VaVSN_l_RefObjCoord[RtUpr][Xcell] = K_Vision.KeVSN_l_RefTgtTop;
      VaVSN_l_RefObjCoord[RtUpr][Ycell] = triHeight;
      VaVSN_l_RefObjCoord[RtUpr][Zcell] = 0.0;

      /*  Determine Point Coordinates: RtLwr Point */
      VaVSN_l_RefObjCoord[RtLwr][Xcell] = K_Vision.KeVSN_l_RefTgtBtm;
      VaVSN_l_RefObjCoord[RtLwr][Ycell] = 0.0;
      VaVSN_l_RefObjCoord[RtLwr][Zcell] = 0.0;
    
      /*  Determine Point Coordinates: LtLwr Point */
      VaVSN_l_RefObjCoord[LtLwr][Xcell] = 0.0;
      VaVSN_l_RefObjCoord[LtLwr][Ycell] = 0.0;
      VaVSN_l_RefObjCoord[LtLwr][Zcell] = 0.0;

      /*  Determine Point Coordinates: LtUpr Point */
      VaVSN_l_RefObjCoord[LtUpr][Xcell] = triBase;
      VaVSN_l_RefObjCoord[LtUpr][Ycell] = triHeight;
      VaVSN_l_RefObjCoord[LtUpr][Zcell] = 0.0;


      /* Build Object Matrix from Array */
      for (i=0;i<NumPts;i++) {
        VmVSN_l_RefObj.put(i,Xcell,VaVSN_l_RefObjCoord[i][Xcell]);
        VmVSN_l_RefObj.put(i,Ycell,VaVSN_l_RefObjCoord[i][Ycell]);
        VmVSN_l_RefObj.put(i,Zcell,VaVSN_l_RefObjCoord[i][Zcell]);
      }
    }


   /**
    * Method: calcVSN_RefTgtImgMat - Load the coordinates of the target
    * image in the image plane in pixel units from calibrations and build
    * the Reference Target Image Matrix.
    */
    private void calcVSN_RefTgtImgMat() {
      int i;

      /* Build Object Matrix from Array */
      for (i=0;i<NumPts;i++) {
        VmVSN_Pxl_RefImg.put(i,Xcell,K_Vision.KaVSN_Pxl_RefImgCoord[i][Xcell]);
        VmVSN_Pxl_RefImg.put(i,Ycell,K_Vision.KaVSN_Pxl_RefImgCoord[i][Ycell]);
      }
    }



   /**
    * Method: calcVSN_CamMat - Calculate the Camera Matrix from the
    * camera Focal Point Fx and Fy and the camera optical center
    * Cx and Cy.
    *                 | Fx,  0, Cx |
    *  Cam Matrix =   |  0, Fy, Cy |
    *                 |  0,  0,  1 |
    */
    private void calcVSN_CamMat() {
      double triBase, triHeight;
      int i;
     
      /* Calculate the Equal-Angular Rhombas End-Triangle Base Lengths */ 
      triBase = (Math.abs(K_Vision.KeVSN_l_RefTgtBtm - K_Vision.KeVSN_l_RefTgtTop))/2;
  
      /* Calculate the Rhombas Height via calculating the Height of the Eng-Triangles. */
      triHeight = Math.sqrt(Math.pow(K_Vision.KeVSN_l_RefTgtSides,2) - Math.pow(triBase,2));
    

      /*  Construct Row0 of Camera Matrix */
      VaVSN_Pxl_CamMatrix[0][0] = VeVSN_Pxl_CamFocalPt;
      VaVSN_Pxl_CamMatrix[0][1] = 0;
      VaVSN_Pxl_CamMatrix[0][2] = K_Vision.KaVSN_Pxl_CamDim[Xcell]/2;

      /*  Construct Row1 of Camera Matrix */
      VaVSN_Pxl_CamMatrix[1][0] = 0;
      VaVSN_Pxl_CamMatrix[1][1] = VeVSN_Pxl_CamFocalPt;
      VaVSN_Pxl_CamMatrix[1][2] = K_Vision.KaVSN_Pxl_CamDim[Ycell]/2;

      /*  Construct Row2 of Camera Matrix */
      VaVSN_Pxl_CamMatrix[2][0] = 0;
      VaVSN_Pxl_CamMatrix[2][1] = 0;
      VaVSN_Pxl_CamMatrix[2][2] = 1;


      /* Build Object Matrix from Array */
      for (i=0;i<3;i++) {
        VmVSN_Pxl_Cam.put(i,0,VaVSN_Pxl_CamMatrix[i][0]);
        VmVSN_Pxl_Cam.put(i,1,VaVSN_Pxl_CamMatrix[i][1]);
        VmVSN_Pxl_Cam.put(i,2,VaVSN_Pxl_CamMatrix[i][2]);
      }

    }  


   /**
    * Method: calcVSN_CamTgtImgGeometry - Calculate the dimensions
    * of the vision target in pixels, i.e. the length of all the sides.
    */
    private void calcVSN_CamTgtImgGeometry() {
      VeVSN_Pxl_ImgWidthTop = calcVSN_PxlLengthLineSeg(VaVSN_Pxl_CamImgCoord[RtUpr], VaVSN_Pxl_CamImgCoord[LtUpr]);
      VeVSN_Pxl_ImgWidthBtm = calcVSN_PxlLengthLineSeg(VaVSN_Pxl_CamImgCoord[RtLwr], VaVSN_Pxl_CamImgCoord[LtLwr]);
      VeVSN_Pxl_ImgHeightLt = calcVSN_PxlLengthLineSeg(VaVSN_Pxl_CamImgCoord[LtLwr], VaVSN_Pxl_CamImgCoord[LtUpr]);
      VeVSN_Pxl_ImgHeightRt = calcVSN_PxlLengthLineSeg(VaVSN_Pxl_CamImgCoord[RtLwr], VaVSN_Pxl_CamImgCoord[RtUpr]);
    }

    
   /**
    * Method: calcVSN_PxlLengthLineSeg - Calculate the length of a
    * line segment given the X-Y coordinates of the two end-points
    * of the line.  Uses the Pythagorean Theorem.
    * @param1: Array of Cartesian Coordinates of Line Segment PointA (int)
    * @param2: Array of Cartesian Coordinates of Line Segment PointB (int)
    * @return: Length of Line Segment in Camera Pixels (double)
    */
    private double calcVSN_PxlLengthLineSeg(int LeVSN_Pxl_PntA[], int LeVSN_Pxl_PntB[]) {
      double diffX, diffY, hyp2;
      double length;
  
      /* Calculate the delta between the points in the X-dimension. */
      diffX = (double)(LeVSN_Pxl_PntA[Xcell] - LeVSN_Pxl_PntB[Xcell]);
  
      /* Calculate the delta between the points in the Y-dimension. */
      diffY = (double)(LeVSN_Pxl_PntA[Ycell] - LeVSN_Pxl_PntB[Ycell]);
  
      /* calculate the hypoteneuse */
      hyp2 = (Math.pow(diffX,2)) + (Math.pow(diffY,2));
      length = Math.sqrt((double)hyp2);

      return (length);
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
    LeVSN_l_RefTgtBtm = K_Vision.KeVSN_l_RefTgtBtm;
    if (LeVSN_l_RefTgtBtm <= 1) {
      LeVSN_l_RefTgtBtm = 1;
    }
      
    LeVSN_l_FocalPt = ((double)K_Vision.KeVSN_Cnt_PxlTgtBtm * K_Vision.KeVSN_l_RefTgtToCamDist)/
                      (LeVSN_l_RefTgtBtm);

	  VeVSN_Pxl_CamFocalPt = (int)LeVSN_l_FocalPt;          
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
   private void calcVSN_TgtDist() {
     double LeVSN_Pxl_In;
     double LeVSN_Pxl_Cam2Tgt;

     LeVSN_Pxl_In = VeVSN_Pxl_ImgWidthBtm;

     /* To protect against a divide by zero error */
     if (LeVSN_Pxl_In < 1) { 
         LeVSN_Pxl_In = 1;
      }
        
     LeVSN_Pxl_Cam2Tgt = (K_Vision.KeVSN_l_RefTgtBtm * VeVSN_Pxl_CamFocalPt)/
                          LeVSN_Pxl_In;

     VeVSN_l_Cam2Tgt2ndry = LeVSN_Pxl_Cam2Tgt;
    }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
