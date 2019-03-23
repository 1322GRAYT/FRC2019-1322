/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Dashboard extends Subsystem {



  // Called to Update SmartDash Data for Display
  public void updateSmartDashCamData() {
    SmartDashboard.putNumber("LL Tgt Vld : ", Robot.VISION.GetVSN_b_LL_TgtVld());
    SmartDashboard.putNumber("LL Tgt Ang-X : ", Robot.VISION.GetVSN_deg_LL_TgtAngX());
    SmartDashboard.putNumber("LL Tgt Ang-Y : ", Robot.VISION.GetVSN_deg_LL_TgtAngY());
    SmartDashboard.putNumber("LL Tgt Area : ", Robot.VISION.GetVSN_Pct_LL_TgtArea());
    SmartDashboard.putNumber("LL Tgt Skew : ", Robot.VISION.GetVSN_Deg_LL_TgtSkew());
    SmartDashboard.putNumber("LL Tgt Side Short : ", Robot.VISION.GetVSN_Pxl_LL_TgtSideShort());
    SmartDashboard.putNumber("LL Tgt Side Long : ", Robot.VISION.GetVSN_Pxl_LL_TgtSideLong());
    SmartDashboard.putNumber("LL Tgt Lngth Hort : ", Robot.VISION.GetVSN_Pxl_LL_TgtLngthHort());
    SmartDashboard.putNumber("LL Tgt Lngth Vert : ", Robot.VISION.GetVSN_Pxl_LL_TgtLngthVert());
    SmartDashboard.putNumber("LL Corner RtU X : ", Robot.VISION.GetVSN_Pxl_LL_TgtCornX(0));
    SmartDashboard.putNumber("LL Corner RtU Y : ", Robot.VISION.GetVSN_Pxl_LL_TgtCornY(0));
    SmartDashboard.putNumber("LL Corner RtL X : ", Robot.VISION.GetVSN_Pxl_LL_TgtCornX(1));
    SmartDashboard.putNumber("LL Corner RtL Y : ", Robot.VISION.GetVSN_Pxl_LL_TgtCornY(1));
    SmartDashboard.putNumber("LL Corner LtL X : ", Robot.VISION.GetVSN_Pxl_LL_TgtCornX(2));
    SmartDashboard.putNumber("LL Corner LtL Y : ", Robot.VISION.GetVSN_Pxl_LL_TgtCornY(2));
    SmartDashboard.putNumber("LL Corner LtR X : ", Robot.VISION.GetVSN_Pxl_LL_TgtCornX(3));
    SmartDashboard.putNumber("LL Corner LtR Y : ", Robot.VISION.GetVSN_Pxl_LL_TgtCornY(3));
    SmartDashboard.putNumber("Cam Img Width Btm (pixels) : ", Robot.VISION.GetVSN_Pxl_ImgWidthBtm());
    SmartDashboard.putNumber("Cam Img Width Top (pixels) : ", Robot.VISION.GetVSN_Pxl_ImgWidthTop());
    SmartDashboard.putNumber("Cam Img Height Lt (pixels) : ", Robot.VISION.GetVSN_Pxl_ImgHeightLt());
    SmartDashboard.putNumber("Cam Img Height Rt (pixels) : ", Robot.VISION.GetVSN_Pxl_ImgHeightRt());
    SmartDashboard.putNumber("Cam Focal Length (pixels) : ", Robot.VISION.GetVSN_Pxl_CamFocalPt());
    SmartDashboard.putNumber("Tgt Cam Dist :    ", Robot.VISION.GetVSN_l_Cam2Tgt());
    SmartDashboard.putNumber("Tgt Cam Dist2 :   ", Robot.VISION.GetVSN_l_Cam2Tgt2ndry());
    SmartDashboard.putNumber("Tgt Rbt Dist :    ", Robot.VISION.GetVSN_l_Rbt2Tgt());
    SmartDashboard.putNumber("Tgt Rbt2Tgt Ang : ", Robot.VISION.GetVSN_deg_Rbt2Tgt());
    SmartDashboard.putNumber("Tgt Rbt2Tgt Rot : ", Robot.VISION.GetVSN_Deg_RbtRot());

    System.out.println("LL Tgt Vld : " + Robot.VISION.GetVSN_b_LL_TgtVld());
    System.out.println("LL Tgt Ang-X : " + Robot.VISION.GetVSN_deg_LL_TgtAngX());
    System.out.println("LL Tgt Ang-Y : " + Robot.VISION.GetVSN_deg_LL_TgtAngY());
    System.out.println("LL Tgt Area : " + Robot.VISION.GetVSN_Pct_LL_TgtArea());
    System.out.println("LL Tgt Skew : " + Robot.VISION.GetVSN_Deg_LL_TgtSkew());
    System.out.println("LL Tgt Side Short : " + Robot.VISION.GetVSN_Pxl_LL_TgtSideShort());
    System.out.println("LL Tgt Side Long : " + Robot.VISION.GetVSN_Pxl_LL_TgtSideLong());
    System.out.println("LL Tgt Lngth Hort : " + Robot.VISION.GetVSN_Pxl_LL_TgtLngthHort());
    System.out.println("LL Tgt Lngth Vert : " + Robot.VISION.GetVSN_Pxl_LL_TgtLngthVert());
    System.out.println("LL Corner RtU X : " + Robot.VISION.GetVSN_Pxl_LL_TgtCornX(0));
    System.out.println("LL Corner RtU Y : " + Robot.VISION.GetVSN_Pxl_LL_TgtCornY(0));
    System.out.println("LL Corner RtL X : " + Robot.VISION.GetVSN_Pxl_LL_TgtCornX(1));
    System.out.println("LL Corner RtL Y : " + Robot.VISION.GetVSN_Pxl_LL_TgtCornY(1));
    System.out.println("LL Corner LtL X : " + Robot.VISION.GetVSN_Pxl_LL_TgtCornX(2));
    System.out.println("LL Corner LtL Y : " + Robot.VISION.GetVSN_Pxl_LL_TgtCornY(2));
    System.out.println("LL Corner LtU X : " + Robot.VISION.GetVSN_Pxl_LL_TgtCornX(3));
    System.out.println("LL Corner LtU Y : " + Robot.VISION.GetVSN_Pxl_LL_TgtCornY(3));
    System.out.println("Cam Img Width Btm (pixels) : " + Robot.VISION.GetVSN_Pxl_ImgWidthBtm());
    System.out.println("Cam Img Width Top (pixels) : " + Robot.VISION.GetVSN_Pxl_ImgWidthTop());
    System.out.println("Cam Img Height Lt (pixels) : " + Robot.VISION.GetVSN_Pxl_ImgHeightLt());
    System.out.println("Cam Img Height Rt (pixels) : " + Robot.VISION.GetVSN_Pxl_ImgHeightRt());
    System.out.println("Cam Focal Length (pixels) :  " + Robot.VISION.GetVSN_Pxl_CamFocalPt());
    System.out.println("Tgt Cam Dist :    " + Robot.VISION.GetVSN_l_Cam2Tgt());
    System.out.println("Tgt Cam Dist2 :   " + Robot.VISION.GetVSN_l_Cam2Tgt2ndry());
    System.out.println("Tgt Rbt Dist :    " + Robot.VISION.GetVSN_l_Rbt2Tgt());
    System.out.println("Tgt Rbt2Tgt Ang : " + Robot.VISION.GetVSN_deg_Rbt2Tgt());
    System.out.println("Tgt Rbt2Tgt Rot : " + Robot.VISION.GetVSN_Deg_RbtRot());
  }




  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
