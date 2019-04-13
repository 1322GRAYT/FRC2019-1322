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
import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.*;
import frc.robot.calibrations.K_System;
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
  private double LL_TgtLngthHorz;  // Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  private double LL_TgtLngthVert;  // Vertical sidelength of the rough bounding box (0 - 320 pixels)
  private double LL_TgtCornX[] = new double[4];  // Target Corner Coord-X: RH, RL, LL, LH
  private double LL_TgtCornY[] = new double[4];  // Target Corner Coord-Y: RH, RL, LL, LH

  /* Array Indexes for building Matricies for Vision Pose Calculations */
  private static final int Xcell  = 0; // X-Cell for array Indexing;
  private static final int Ycell  = 1; // Y-Cell for array Indexing;
  private static final int Zcell  = 2; // Z-Cell for array Indexing;

  private static final int RtUpr  = 0;  // Top Right Cell for array Indexing;
  private static final int RtLwr  = 1;  // Bottom Right Cell for array Indexing;
  private static final int LtLwr  = 2;  // Bottom Left Cell for array Indexing;
  private static final int LtUpr  = 3;  // Top Left Cell for array Indexing;
  private static final int NumPts = 4;  // Total Number of Data Points;

  /* Array of the Matrix Data for viewing via Instrumentation */
  private int VaVSN_Pxl_CamImgCoord[][] = new int[4][2];

  private int VeVSN_Cnt_TgtCornAqrd;



  public Vision() {
    /* Empty Constructor */
  }

  /**********************************************/
  /* Matricies for Object Dimensions and Images */
  /**********************************************/

  /* Active CameraServer Camera */
  private int Active_Camera = 0; // Default to camera 0


  /*******************************/
  /* Public Class Interfaces     */
  /*******************************/

 /**
   * Method: getVSN_b_LL_TgtVld - Indication that Valid Target data has
   * been acquired.
   * @return: double - boolean (valid = true)
   */
  public double getVSN_b_LL_TgtVld() {
    return(LL_TgtVld);
  }

 /**
   * Method: getVSN_Deg_LL_TgtAngX - Horizontal Offset from CrossHair to
   * Target (-27 to 27 degrees).
   * @return: double - degrees
   */
  public double getVSN_Deg_LL_TgtAngX() {
    return(LL_TgtAngX);
  }

 /**
   * Method: getVSN_Deg_LL_TgtAngY - Vertical Offset from CrossHair to
   * Target (-20.5 to 20.5 degrees).
   * @return: double - degrees
   */
  public double getVSN_Deg_LL_TgtAngY() {
    return(LL_TgtAngY);
  }

 /**
   * Method: getVSN_Pct_LL_TgtArea - Target Area (0% of image to 100% of image).
   * @return: double - percent
   */
  public double getVSN_Pct_LL_TgtArea() {
    return(LL_TgtArea);
  }

 /**
   * Method: getVSN_Deg_LL_TgtSkew - Target Skew or Rotation
   * @return: double - degrees
   */
  public double getVSN_Deg_LL_TgtSkew() {
    return(LL_TgtSkew);
  }

 /**
   * Method: getVSN_Pxl_LL_TgtSideShort - Sidelength of shortest side
   * of the fitted bounding box (pixels).  
   * @return: double - pixels
   */
public double getVSN_Pxl_LL_TgtSideShort() {
    return(LL_TgtSideShort);
  }

 /**
   * Method: getVSN_Pxl_LL_TgtSideLong - Sidelength of longest side
   * of the fitted bounding box (pixels).  
   * @return: double - pixels
   */
  public double getVSN_Pxl_LL_TgtSideLong() {
    return(LL_TgtSideLong);
  }

 /**
   * Method: getVSN_Pxl_LL_TgtLngthHorz - Horizontal sidelength of
   * the rough bounding box (0 - 320 pixels).
   * @return: double - pixels
   */
  public double getVSN_Pxl_LL_TgtLngthHorz() {
    return(LL_TgtLngthHorz);
  }

 /**
   * Method: getVSN_Pxl_LL_TgtLngthVert - Vertical sidelength of
   * the rough bounding box (0 - 320 pixels).  
   * @return: double - pixels
   */
  public double getVSN_Pxl_LL_TgtLngthVert() {
    return(LL_TgtLngthVert);
  }

 /**
   * Method: getVSN_Pxl_LL_TgtCornX - Target Corner X-Coordinates
   * RH, RL, LL, LH (0 - 320 pixels).
   * @param1: int - Xcell index (RH, RL, LL, LH)
   * @return: double - pixels X Coordinates (RH, RL, LL, LH)
   */
  public double getVSN_Pxl_LL_TgtCornX(int LeVSN_i_CellIdx) {
    return(LL_TgtCornX[LeVSN_i_CellIdx]);
  }

// Target Corner Coord-Y: RH, RL, LL, LH (0 - 320 pixels)
 /**
   * Method: getVSN_Pxl_LL_TgtCornY - Target Corner Y-Coordinates
   * RH, RL, LL, LH (0 - 320 pixels).
   * @param1: int - Ycell index (RH, RL, LL, LH)
   * @return: double - pixels Y Coordinates (RH, RL, LL, LH)
   */
  public double getVSN_Pxl_LL_TgtCornY(int LeVSN_i_CellIdx) {
    return(LL_TgtCornY[LeVSN_i_CellIdx]);
  }


  /*******************************/
  /* Public Class Methods        */
  /*******************************/

   /**
    * Method: mngVSN_InitLimeLightNetTbl - Processes the Initialization
    * of the Network Tables Address Mapping for the Lime Light Camera.
    */
    public void mngVSN_InitLimeLightNetTbl() {
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
    * Method: mngVSN_CamImgPeriodic - Manage the Periodic Cameara
    * Image processing tasks.
    */
    public void mngVSN_CamImgPeriodic() {
      captureVSN_CamImgData();
    }
  
 
  /**
    * Method: dtrmnVSN_CamVldData - Determine whether the Raw Image Data from
    * the Camera is valid to calculate the Image Pose data.
    */
    public boolean dtrmnVSN_CamVldData() {
      boolean LeVSN_b_TgtAcqVld = false;

      VeVSN_Cnt_TgtCornAqrd = LL_TgtCornX.length;

      if ((LL_TgtVld == 1.0) &&
          (LL_TgtCornX.length >= K_Vision.KeVSN_Cnt_CamTgtCornMin) &&
          (LL_TgtCornY.length >= K_Vision.KeVSN_Cnt_CamTgtCornMin) &&
          (LL_TgtCornX.length <= 4)) {
        LeVSN_b_TgtAcqVld = true;
        VeVSN_Cnt_TgtCornAqrd = LL_TgtCornX.length;

        System.out.println("VeVSN_Cnt_TgtCornAqrd : " + VeVSN_Cnt_TgtCornAqrd);
        
        if ((K_System.KeSYS_e_DebugEnblVsn == DebugSlct.DebugEnblBoth) ||
        (K_System.KeSYS_e_DebugEnblVsn == DebugSlct.DebugEnblSDB)) {
          Robot.DASHBOARD.updINS_SDB_RawLL_Data();
        }
        if ((K_System.KeSYS_e_DebugEnblVsn == DebugSlct.DebugEnblBoth) ||
            (K_System.KeSYS_e_DebugEnblVsn == DebugSlct.DebugEnblRRL)) {
          Robot.DASHBOARD.updINS_RRL_RawLL_Data();
        }          

      }
      else { /* (LeVSN_b_TgtAcqVld == false) */
        System.out.println("Waiting for Valid 3+ Corner data-image ... ");
      }
        
      return(LeVSN_b_TgtAcqVld);
    }


  /**
    * Method: MngVSN_CamImgProc - Calculate the dimensions
    * of the vision target in pixels, i.e. the length of all the sides.
    */
    public void MngVSN_CamImgProc() {
      parseVSN_CamImgData();
    }


    /**
     * Method: SlctVSN_DrvrCam - Switches between the Visiual Driver-View cameras.
     */ 
    public void SlctVSN_DrvrCam() {
      if(this.Active_Camera == 0) { // Camera is 0, set to 1
        this.NetTbl.getTable("").getEntry("CameraSelection").setString(Robot.camera1.getName());
        this.Active_Camera = 1;
      } else { // Camera is 1, set to 0
        this.NetTbl.getTable("").getEntry("CameraSelection").setString(Robot.camera0.getName());
        this.Active_Camera = 0;
      }
      
    }



  /*******************************/
  /* Internal Class Methods      */
  /*******************************/   
    
  /**
    * Method: captureVSN_CamImgData - Capture the Raw Image Data from
    * the Camera.  Receives the data from the
    * Network Tables that have been transmitted from the Rpi Controller.
    */
    private void captureVSN_CamImgData() {
      //read values periodically
      LL_TgtVld  = tv.getDouble(0.0);
      LL_TgtAngX = tx.getDouble(0.0);
      LL_TgtAngY = ty.getDouble(0.0);
      LL_TgtArea = ta.getDouble(0.0);
      LL_TgtSkew = ts.getDouble(0.0);
      LL_TgtSideShort = tshort.getDouble(0.0);
      LL_TgtSideLong  = tlong.getDouble(0.0);
      LL_TgtLngthHorz = thor.getDouble(0.0);
      LL_TgtLngthVert = tvert.getDouble(0.0);
      LL_TgtCornX   = tcornx.getDoubleArray(new double[0]);
      LL_TgtCornY   = tcorny.getDoubleArray(new double[0]);
    }



  /**
    * Method: parseVSN_CamImgData - Update the Raw Image Data from
    * the Camera and loading it into the proper arrays for Target
    * Distance and Angle processing.
    */
    private void parseVSN_CamImgData() {
      int i;

      for (i=0;i<NumPts;i++) {
        VaVSN_Pxl_CamImgCoord[i][Xcell] = (int)LL_TgtCornX[i];
        VaVSN_Pxl_CamImgCoord[i][Ycell] = (int)LL_TgtCornY[i];
      }
    }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
