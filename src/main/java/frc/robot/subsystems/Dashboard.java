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


  /* Called to Update SmartDash Data for Display of Vision Data */
  public void updateSmartDashCamData() {
    SmartDashboard.putNumber("LL Tgt Vld : ", Robot.VISION.getVSN_b_LL_TgtVld());
    SmartDashboard.putNumber("LL Tgt Ang-X : ", Robot.VISION.getVSN_deg_LL_TgtAngX());
    SmartDashboard.putNumber("LL Tgt Ang-Y : ", Robot.VISION.getVSN_deg_LL_TgtAngY());
    SmartDashboard.putNumber("LL Tgt Area : ", Robot.VISION.getVSN_Pct_LL_TgtArea());
    SmartDashboard.putNumber("LL Tgt Skew : ", Robot.VISION.getVSN_Deg_LL_TgtSkew());
    SmartDashboard.putNumber("LL Tgt Side Short : ", Robot.VISION.getVSN_Pxl_LL_TgtSideShort());
    SmartDashboard.putNumber("LL Tgt Side Long : ", Robot.VISION.getVSN_Pxl_LL_TgtSideLong());
    SmartDashboard.putNumber("LL Tgt Lngth Hort : ", Robot.VISION.getVSN_Pxl_LL_TgtLngthHort());
    SmartDashboard.putNumber("LL Tgt Lngth Vert : ", Robot.VISION.getVSN_Pxl_LL_TgtLngthVert());
    SmartDashboard.putNumber("LL Corner RtU X : ", Robot.VISION.getVSN_Pxl_LL_TgtCornX(0));
    SmartDashboard.putNumber("LL Corner RtU Y : ", Robot.VISION.getVSN_Pxl_LL_TgtCornY(0));
    SmartDashboard.putNumber("LL Corner RtL X : ", Robot.VISION.getVSN_Pxl_LL_TgtCornX(1));
    SmartDashboard.putNumber("LL Corner RtL Y : ", Robot.VISION.getVSN_Pxl_LL_TgtCornY(1));
    SmartDashboard.putNumber("LL Corner LtL X : ", Robot.VISION.getVSN_Pxl_LL_TgtCornX(2));
    SmartDashboard.putNumber("LL Corner LtL Y : ", Robot.VISION.getVSN_Pxl_LL_TgtCornY(2));
    SmartDashboard.putNumber("LL Corner LtR X : ", Robot.VISION.getVSN_Pxl_LL_TgtCornX(3));
    SmartDashboard.putNumber("LL Corner LtR Y : ", Robot.VISION.getVSN_Pxl_LL_TgtCornY(3));
    SmartDashboard.putNumber("Cam Img Width Btm (pixels) : ", Robot.VISION.getVSN_Pxl_ImgWidthBtm());
    SmartDashboard.putNumber("Cam Img Width Top (pixels) : ", Robot.VISION.getVSN_Pxl_ImgWidthTop());
    SmartDashboard.putNumber("Cam Img Height Lt (pixels) : ", Robot.VISION.getVSN_Pxl_ImgHeightLt());
    SmartDashboard.putNumber("Cam Img Height Rt (pixels) : ", Robot.VISION.getVSN_Pxl_ImgHeightRt());
    SmartDashboard.putNumber("Cam Focal Length (pixels) : ", Robot.VISION.getVSN_Pxl_CamFocalPt());
    SmartDashboard.putNumber("Tgt Cam Dist :    ", Robot.VISION.getVSN_l_Cam2Tgt());
    SmartDashboard.putNumber("Tgt Cam Dist2 :   ", Robot.VISION.getVSN_l_Cam2Tgt2ndry());
    SmartDashboard.putNumber("Tgt Rbt Dist :    ", Robot.VISION.getVSN_l_Rbt2Tgt());
    SmartDashboard.putNumber("Tgt Rbt2Tgt Ang : ", Robot.VISION.getVSN_deg_Rbt2Tgt());
    SmartDashboard.putNumber("Tgt Rbt2Tgt Rot : ", Robot.VISION.getVSN_Deg_RbtRot());

    System.out.println("LL Tgt Vld : " + Robot.VISION.getVSN_b_LL_TgtVld());
    System.out.println("LL Tgt Ang-X : " + Robot.VISION.getVSN_deg_LL_TgtAngX());
    System.out.println("LL Tgt Ang-Y : " + Robot.VISION.getVSN_deg_LL_TgtAngY());
    System.out.println("LL Tgt Area : " + Robot.VISION.getVSN_Pct_LL_TgtArea());
    System.out.println("LL Tgt Skew : " + Robot.VISION.getVSN_Deg_LL_TgtSkew());
    System.out.println("LL Tgt Side Short : " + Robot.VISION.getVSN_Pxl_LL_TgtSideShort());
    System.out.println("LL Tgt Side Long : " + Robot.VISION.getVSN_Pxl_LL_TgtSideLong());
    System.out.println("LL Tgt Lngth Hort : " + Robot.VISION.getVSN_Pxl_LL_TgtLngthHort());
    System.out.println("LL Tgt Lngth Vert : " + Robot.VISION.getVSN_Pxl_LL_TgtLngthVert());
    System.out.println("LL Corner RtU X : " + Robot.VISION.getVSN_Pxl_LL_TgtCornX(0));
    System.out.println("LL Corner RtU Y : " + Robot.VISION.getVSN_Pxl_LL_TgtCornY(0));
    System.out.println("LL Corner RtL X : " + Robot.VISION.getVSN_Pxl_LL_TgtCornX(1));
    System.out.println("LL Corner RtL Y : " + Robot.VISION.getVSN_Pxl_LL_TgtCornY(1));
    System.out.println("LL Corner LtL X : " + Robot.VISION.getVSN_Pxl_LL_TgtCornX(2));
    System.out.println("LL Corner LtL Y : " + Robot.VISION.getVSN_Pxl_LL_TgtCornY(2));
    System.out.println("LL Corner LtU X : " + Robot.VISION.getVSN_Pxl_LL_TgtCornX(3));
    System.out.println("LL Corner LtU Y : " + Robot.VISION.getVSN_Pxl_LL_TgtCornY(3));
    System.out.println("Cam Img Width Btm (pixels) : " + Robot.VISION.getVSN_Pxl_ImgWidthBtm());
    System.out.println("Cam Img Width Top (pixels) : " + Robot.VISION.getVSN_Pxl_ImgWidthTop());
    System.out.println("Cam Img Height Lt (pixels) : " + Robot.VISION.getVSN_Pxl_ImgHeightLt());
    System.out.println("Cam Img Height Rt (pixels) : " + Robot.VISION.getVSN_Pxl_ImgHeightRt());
    System.out.println("Cam Focal Length (pixels) :  " + Robot.VISION.getVSN_Pxl_CamFocalPt());
    System.out.println("Tgt Cam Dist :    " + Robot.VISION.getVSN_l_Cam2Tgt());
    System.out.println("Tgt Cam Dist2 :   " + Robot.VISION.getVSN_l_Cam2Tgt2ndry());
    System.out.println("Tgt Rbt Dist :    " + Robot.VISION.getVSN_l_Rbt2Tgt());
    System.out.println("Tgt Rbt2Tgt Ang : " + Robot.VISION.getVSN_deg_Rbt2Tgt());
    System.out.println("Tgt Rbt2Tgt Rot : " + Robot.VISION.getVSN_Deg_RbtRot());
  }


  /* Called to Update SmartDash Data for Display of Drive Data */
  public void updateSmartDashDrvData() {
    SmartDashboard.putNumber("Encoder Cnts FL : ", Robot.DRIVES.getDRV_cnt_EncdrCnt(0));
    SmartDashboard.putNumber("Encoder Cnts FR : ", Robot.DRIVES.getDRV_cnt_EncdrCnt(1));
    SmartDashboard.putNumber("Encoder Cnts RL : ", Robot.DRIVES.getDRV_cnt_EncdrCnt(2));
    SmartDashboard.putNumber("Encoder Cnts RR : ", Robot.DRIVES.getDRV_cnt_EncdrCnt(3));
    SmartDashboard.putNumber("Encoder Vel Raw FL : ", Robot.DRIVES.getDRV_f_EncdrVelRaw(0));
    SmartDashboard.putNumber("Encoder Vel Raw FR : ", Robot.DRIVES.getDRV_f_EncdrVelRaw(1));
    SmartDashboard.putNumber("Encoder Vel Raw RL : ", Robot.DRIVES.getDRV_f_EncdrVelRaw(2));
    SmartDashboard.putNumber("Encoder Vel Raw RR : ", Robot.DRIVES.getDRV_f_EncdrVelRaw(3));
    SmartDashboard.putNumber("Encoder Vel FL : ", Robot.DRIVES.getDRV_f_EncdrVel(0));
    SmartDashboard.putNumber("Encoder Vel FR : ", Robot.DRIVES.getDRV_f_EncdrVel(1));
    SmartDashboard.putNumber("Encoder Vel RL : ", Robot.DRIVES.getDRV_f_EncdrVel(2));
    SmartDashboard.putNumber("Encoder Vel RR : ", Robot.DRIVES.getDRV_f_EncdrVel(3));
    SmartDashboard.putNumber("Encoder RPM FL : ", Robot.DRIVES.getDRV_n_EncdrRPM(0));
    SmartDashboard.putNumber("Encoder RPM FR : ", Robot.DRIVES.getDRV_n_EncdrRPM(1));
    SmartDashboard.putNumber("Encoder RPM RL : ", Robot.DRIVES.getDRV_n_EncdrRPM(2));
    SmartDashboard.putNumber("Encoder RPM RR : ", Robot.DRIVES.getDRV_n_EncdrRPM(3));
    SmartDashboard.putNumber("Wheel RPM FL : ", Robot.DRIVES.getDRV_n_WhlRPM(0));
    SmartDashboard.putNumber("Wheel RPM FR : ", Robot.DRIVES.getDRV_n_WhlRPM(1));
    SmartDashboard.putNumber("Wheel RPM RL : ", Robot.DRIVES.getDRV_n_WhlRPM(2));
    SmartDashboard.putNumber("Wheel RPM RR : ", Robot.DRIVES.getDRV_n_WhlRPM(3));
    SmartDashboard.putNumber("Wheel Vel FL : ", Robot.DRIVES.getDRV_v_WhlVel(0));
    SmartDashboard.putNumber("Wheel Vel FR : ", Robot.DRIVES.getDRV_v_WhlVel(1));
    SmartDashboard.putNumber("Wheel Vel RL : ", Robot.DRIVES.getDRV_v_WhlVel(2));
    SmartDashboard.putNumber("Wheel Vel RR : ", Robot.DRIVES.getDRV_v_WhlVel(3));

    System.out.println("Encoder Cnts FL : " + Robot.DRIVES.getDRV_cnt_EncdrCnt(0));
    System.out.println("Encoder Cnts FR : " + Robot.DRIVES.getDRV_cnt_EncdrCnt(1));
    System.out.println("Encoder Cnts RL : " + Robot.DRIVES.getDRV_cnt_EncdrCnt(2));
    System.out.println("Encoder Cnts RR : " + Robot.DRIVES.getDRV_cnt_EncdrCnt(3));
    System.out.println("Encoder Vel Raw FL : " + Robot.DRIVES.getDRV_f_EncdrVelRaw(0));
    System.out.println("Encoder Vel Raw FR : " + Robot.DRIVES.getDRV_f_EncdrVelRaw(1));
    System.out.println("Encoder Vel Raw RL : " + Robot.DRIVES.getDRV_f_EncdrVelRaw(2));
    System.out.println("Encoder Vel Raw RR : " + Robot.DRIVES.getDRV_f_EncdrVelRaw(3));
    System.out.println("Encoder Vel FL : " + Robot.DRIVES.getDRV_f_EncdrVel(0));
    System.out.println("Encoder Vel FR : " + Robot.DRIVES.getDRV_f_EncdrVel(1));
    System.out.println("Encoder Vel RL : " + Robot.DRIVES.getDRV_f_EncdrVel(2));
    System.out.println("Encoder Vel RR : " + Robot.DRIVES.getDRV_f_EncdrVel(3));
    System.out.println("Encoder RPM FL : " + Robot.DRIVES.getDRV_n_EncdrRPM(0));
    System.out.println("Encoder RPM FR : " + Robot.DRIVES.getDRV_n_EncdrRPM(1));
    System.out.println("Encoder RPM RL : " + Robot.DRIVES.getDRV_n_EncdrRPM(2));
    System.out.println("Encoder RPM RR : " + Robot.DRIVES.getDRV_n_EncdrRPM(3));
    System.out.println("Wheel RPM FL : " + Robot.DRIVES.getDRV_n_WhlRPM(0));
    System.out.println("Wheel RPM FR : " + Robot.DRIVES.getDRV_n_WhlRPM(1));
    System.out.println("Wheel RPM RL : " + Robot.DRIVES.getDRV_n_WhlRPM(2));
    System.out.println("Wheel RPM RR : " + Robot.DRIVES.getDRV_n_WhlRPM(3));
    System.out.println("Wheel Vel FL : " + Robot.DRIVES.getDRV_v_WhlVel(0));
    System.out.println("Wheel Vel FR : " + Robot.DRIVES.getDRV_v_WhlVel(1));
    System.out.println("Wheel Vel RL : " + Robot.DRIVES.getDRV_v_WhlVel(2));
    System.out.println("Wheel Vel RR : " + Robot.DRIVES.getDRV_v_WhlVel(3));
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
