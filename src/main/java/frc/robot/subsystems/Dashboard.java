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

  enum DebugSlct {
    DebugDsbl, DebugEnblDash, DebugEnblSmartDash, DebugEnblBoth
  }


  /* Called to Update Smart Dashboard Data for Display of Drive Navigation Drive Data */
  public void updINS_SDB_NAV_Drv() { 
    SmartDashboard.putNumber("Encoder Cnts FL : ", Robot.NAV.getNAV_cnt_EncdrCnt(0));
    SmartDashboard.putNumber("Encoder Cnts FR : ", Robot.NAV.getNAV_cnt_EncdrCnt(1));
    SmartDashboard.putNumber("Encoder Cnts RL : ", Robot.NAV.getNAV_cnt_EncdrCnt(2));
    SmartDashboard.putNumber("Encoder Cnts RR : ", Robot.NAV.getNAV_cnt_EncdrCnt(3));
    SmartDashboard.putNumber("Encoder Vel FL : ", Robot.NAV.getNAV_f_EncdrVel(0));
    SmartDashboard.putNumber("Encoder Vel FR : ", Robot.NAV.getNAV_f_EncdrVel(1));
    SmartDashboard.putNumber("Encoder Vel RL : ", Robot.NAV.getNAV_f_EncdrVel(2));
    SmartDashboard.putNumber("Encoder Vel RR : ", Robot.NAV.getNAV_f_EncdrVel(3));
    SmartDashboard.putNumber("Encoder RPM FL : ", Robot.NAV.getNAV_n_EncdrRPM(0));
    SmartDashboard.putNumber("Encoder RPM FR : ", Robot.NAV.getNAV_n_EncdrRPM(1));
    SmartDashboard.putNumber("Encoder RPM RL : ", Robot.NAV.getNAV_n_EncdrRPM(2));
    SmartDashboard.putNumber("Encoder RPM RR : ", Robot.NAV.getNAV_n_EncdrRPM(3));
    SmartDashboard.putNumber("Wheel RPM FL : ", Robot.NAV.getNAV_n_WhlRPM(0));
    SmartDashboard.putNumber("Wheel RPM FR : ", Robot.NAV.getNAV_n_WhlRPM(1));
    SmartDashboard.putNumber("Wheel RPM RL : ", Robot.NAV.getNAV_n_WhlRPM(2));
    SmartDashboard.putNumber("Wheel RPM RR : ", Robot.NAV.getNAV_n_WhlRPM(3));
    SmartDashboard.putNumber("Wheel Vel FL : ", Robot.NAV.getNAV_v_WhlVel(0));
    SmartDashboard.putNumber("Wheel Vel FR : ", Robot.NAV.getNAV_v_WhlVel(1));
    SmartDashboard.putNumber("Wheel Vel RL : ", Robot.NAV.getNAV_v_WhlVel(2));
    SmartDashboard.putNumber("Wheel Vel RR : ", Robot.NAV.getNAV_v_WhlVel(3));
  }

  /* Called to Update RoboRIO Logfile for Display of Drive Navigation Drive Data */
  public void updINS_RRL_NAV_Drv() { 
    System.out.println("Encoder Cnts FL : " + Robot.NAV.getNAV_cnt_EncdrCnt(0));
    System.out.println("Encoder Cnts FR : " + Robot.NAV.getNAV_cnt_EncdrCnt(1));
    System.out.println("Encoder Cnts RL : " + Robot.NAV.getNAV_cnt_EncdrCnt(2));
    System.out.println("Encoder Cnts RR : " + Robot.NAV.getNAV_cnt_EncdrCnt(3));
    System.out.println("Encoder Vel FL : " + Robot.NAV.getNAV_f_EncdrVel(0));
    System.out.println("Encoder Vel FR : " + Robot.NAV.getNAV_f_EncdrVel(1));
    System.out.println("Encoder Vel RL : " + Robot.NAV.getNAV_f_EncdrVel(2));
    System.out.println("Encoder Vel RR : " + Robot.NAV.getNAV_f_EncdrVel(3));
    System.out.println("Encoder RPM FL : " + Robot.NAV.getNAV_n_EncdrRPM(0));
    System.out.println("Encoder RPM FR : " + Robot.NAV.getNAV_n_EncdrRPM(1));
    System.out.println("Encoder RPM RL : " + Robot.NAV.getNAV_n_EncdrRPM(2));
    System.out.println("Encoder RPM RR : " + Robot.NAV.getNAV_n_EncdrRPM(3));
    System.out.println("Wheel RPM FL : " + Robot.NAV.getNAV_n_WhlRPM(0));
    System.out.println("Wheel RPM FR : " + Robot.NAV.getNAV_n_WhlRPM(1));
    System.out.println("Wheel RPM RL : " + Robot.NAV.getNAV_n_WhlRPM(2));
    System.out.println("Wheel RPM RR : " + Robot.NAV.getNAV_n_WhlRPM(3));
    System.out.println("Wheel Vel FL : " + Robot.NAV.getNAV_v_WhlVel(0));
    System.out.println("Wheel Vel FR : " + Robot.NAV.getNAV_v_WhlVel(1));
    System.out.println("Wheel Vel RL : " + Robot.NAV.getNAV_v_WhlVel(2));
    System.out.println("Wheel Vel RR : " + Robot.NAV.getNAV_v_WhlVel(3));
  }
  


  /* Called to Update SmartDash Data for Display of Vision Camera Calibration Data */
  public void updINS_SDB_CamCal() {
    SmartDashboard.putNumber("Cam Focal Length (pixels) : ", Robot.VISION.getVSN_Pxl_CamFocalPt());
  }
  
  /* Called to Update SmartDash Data for Display of Vision Camera Calibration Data */
  public void updINS_RRL_CamCal() {
    System.out.println("Cam Focal Length (pixels) :  " + Robot.VISION.getVSN_Pxl_CamFocalPt());
  }


  /* Called to Update Smart Dashboard Data for Display of Vision raw Camera Limelight Data */
    public void updINS_SDB_RawLL_Data() {
    SmartDashboard.putNumber("LL Tgt Vld : ", Robot.VISION.getVSN_b_LL_TgtVld());
    SmartDashboard.putNumber("LL Tgt Ang-X : ", Robot.VISION.getVSN_Deg_LL_TgtAngX());
    SmartDashboard.putNumber("LL Tgt Ang-Y : ", Robot.VISION.getVSN_Deg_LL_TgtAngY());
    SmartDashboard.putNumber("LL Tgt Area : ", Robot.VISION.getVSN_Pct_LL_TgtArea());
    SmartDashboard.putNumber("LL Tgt Skew : ", Robot.VISION.getVSN_Deg_LL_TgtSkew());
    SmartDashboard.putNumber("LL Tgt Side Short : ", Robot.VISION.getVSN_Pxl_LL_TgtSideShort());
    SmartDashboard.putNumber("LL Tgt Side Long : ", Robot.VISION.getVSN_Pxl_LL_TgtSideLong());
    SmartDashboard.putNumber("LL Tgt Lngth Horz : ", Robot.VISION.getVSN_Pxl_LL_TgtLngthHorz());
    SmartDashboard.putNumber("LL Tgt Lngth Vert : ", Robot.VISION.getVSN_Pxl_LL_TgtLngthVert());
    SmartDashboard.putNumber("LL Corner RtU X : ", Robot.VISION.getVSN_Pxl_LL_TgtCornX(0));
    SmartDashboard.putNumber("LL Corner RtU Y : ", Robot.VISION.getVSN_Pxl_LL_TgtCornY(0));
    SmartDashboard.putNumber("LL Corner RtL X : ", Robot.VISION.getVSN_Pxl_LL_TgtCornX(1));
    SmartDashboard.putNumber("LL Corner RtL Y : ", Robot.VISION.getVSN_Pxl_LL_TgtCornY(1));
    SmartDashboard.putNumber("LL Corner LtL X : ", Robot.VISION.getVSN_Pxl_LL_TgtCornX(2));
    SmartDashboard.putNumber("LL Corner LtL Y : ", Robot.VISION.getVSN_Pxl_LL_TgtCornY(2));
    SmartDashboard.putNumber("LL Corner LtR X : ", Robot.VISION.getVSN_Pxl_LL_TgtCornX(3));
    SmartDashboard.putNumber("LL Corner LtR Y : ", Robot.VISION.getVSN_Pxl_LL_TgtCornY(3));
  }

/* Called to Update RoboRIO Logfile for Display of Vision raw Camera Limelight Data */
public void updINS_RRL_RawLL_Data() {
    System.out.println("LL Tgt Vld : " + Robot.VISION.getVSN_b_LL_TgtVld());
    System.out.println("LL Tgt Ang-X : " + Robot.VISION.getVSN_Deg_LL_TgtAngX());
    System.out.println("LL Tgt Ang-Y : " + Robot.VISION.getVSN_Deg_LL_TgtAngY());
    System.out.println("LL Tgt Area : " + Robot.VISION.getVSN_Pct_LL_TgtArea());
    System.out.println("LL Tgt Skew : " + Robot.VISION.getVSN_Deg_LL_TgtSkew());
    System.out.println("LL Tgt Side Short : " + Robot.VISION.getVSN_Pxl_LL_TgtSideShort());
    System.out.println("LL Tgt Side Long : " + Robot.VISION.getVSN_Pxl_LL_TgtSideLong());
    System.out.println("LL Tgt Lngth Horz : " + Robot.VISION.getVSN_Pxl_LL_TgtLngthHorz());
    System.out.println("LL Tgt Lngth Vert : " + Robot.VISION.getVSN_Pxl_LL_TgtLngthVert());
    System.out.println("LL Corner RtU X : " + Robot.VISION.getVSN_Pxl_LL_TgtCornX(0));
    System.out.println("LL Corner RtU Y : " + Robot.VISION.getVSN_Pxl_LL_TgtCornY(0));
    System.out.println("LL Corner RtL X : " + Robot.VISION.getVSN_Pxl_LL_TgtCornX(1));
    System.out.println("LL Corner RtL Y : " + Robot.VISION.getVSN_Pxl_LL_TgtCornY(1));
    System.out.println("LL Corner LtL X : " + Robot.VISION.getVSN_Pxl_LL_TgtCornX(2));
    System.out.println("LL Corner LtL Y : " + Robot.VISION.getVSN_Pxl_LL_TgtCornY(2));
    System.out.println("LL Corner LtU X : " + Robot.VISION.getVSN_Pxl_LL_TgtCornX(3));
    System.out.println("LL Corner LtU Y : " + Robot.VISION.getVSN_Pxl_LL_TgtCornY(3));
  }



  /* Called to Update SmartDash Data for Display of Closed-Loop Error Data. */
  public void updINS_SDB_PID_CL_Err() { 
    SmartDashboard.putNumber("VePID_Deg_PstnErr :        ", Robot.PID.getPID_Deg_PstnErr());
    SmartDashboard.putBoolean("VePID_b_PstnErrWithInDB : ", Robot.PID.getPID_b_PstnErrWithInDB());
    SmartDashboard.putNumber("VePID_Deg_PstnErrAccum :   ", Robot.PID.getPID_Deg_ErrAccum());
  }

  /* Called to Update RoboRIO Logfile for Display of Closed=Loop Error Data. */
  public void updINS_RRL_PID_CL_Err() { 
    System.out.println("VePID_Deg_PstnErr :       " + Robot.PID.getPID_Deg_PstnErr());
    System.out.println("VePID_b_PstnErrWithInDB : " + Robot.PID.getPID_b_PstnErrWithInDB());
    System.out.println("VePID_Deg_PstnErrAccum :  " + Robot.PID.getPID_Deg_ErrAccum());
  }


  /* Called to Update SmartDash Data for Display of Navigation System Image Calculations. */
  public void updINS_SDB_VSN_CamImgCalc() {
    SmartDashboard.putNumber("Cam Img Width Btm (pixels) : ", Robot.VISION.getVSN_Pxl_ImgWidthBtm());
    SmartDashboard.putNumber("Cam Img Width Top (pixels) : ", Robot.VISION.getVSN_Pxl_ImgWidthTop());
    SmartDashboard.putNumber("Cam Img Height Lt (pixels) : ", Robot.VISION.getVSN_Pxl_ImgHeightLt());
    SmartDashboard.putNumber("Cam Img Height Rt (pixels) : ", Robot.VISION.getVSN_Pxl_ImgHeightRt());
    SmartDashboard.putNumber("Tgt Cam Dist :    ", Robot.VISION.getVSN_l_Cam2Tgt());
    SmartDashboard.putNumber("Tgt Cam Dist2 :   ", Robot.VISION.getVSN_l_Cam2Tgt2ndry());
    SmartDashboard.putNumber("Tgt Rbt Dist :    ", Robot.VISION.getVSN_l_Rbt2Tgt());
    SmartDashboard.putNumber("Tgt Rbt2Tgt Ang : ", Robot.VISION.getVSN_Deg_Rbt2Tgt());
    SmartDashboard.putNumber("Tgt Rbt2Tgt Rot : ", Robot.VISION.getVSN_Deg_RbtRot());
  }

  /* Called to Update RoboRIO Logfile for Display of Navigation System Image Calculations. */
  public void updINS_RRL_VSN_CamImgCalc() {
    System.out.println("Cam Img Width Btm (pixels) : " + Robot.VISION.getVSN_Pxl_ImgWidthBtm());
    System.out.println("Cam Img Width Top (pixels) : " + Robot.VISION.getVSN_Pxl_ImgWidthTop());
    System.out.println("Cam Img Height Lt (pixels) : " + Robot.VISION.getVSN_Pxl_ImgHeightLt());
    System.out.println("Cam Img Height Rt (pixels) : " + Robot.VISION.getVSN_Pxl_ImgHeightRt());
    System.out.println("Tgt Cam Dist :    " + Robot.VISION.getVSN_l_Cam2Tgt());
    System.out.println("Tgt Cam Dist2 :   " + Robot.VISION.getVSN_l_Cam2Tgt2ndry());
    System.out.println("Tgt Rbt Dist :    " + Robot.VISION.getVSN_l_Rbt2Tgt());
    System.out.println("Tgt Rbt2Tgt Ang : " + Robot.VISION.getVSN_Deg_Rbt2Tgt());
    System.out.println("Tgt Rbt2Tgt Rot : " + Robot.VISION.getVSN_Deg_RbtRot());
  }

  
  
  /* Called to Update Smart Dashboard Data for Display of Target Closed-Loop Calculations  */
  public void updINS_SDB_PID_CorrCalc() { 
    SmartDashboard.putNumber("VePID_Deg_PstnAct :        ", Robot.PID.getPID_Deg_PstnAct());
    SmartDashboard.putNumber("VePID_Deg_PstnErr :        ", Robot.PID.getPID_Deg_PstnErr());
    SmartDashboard.putBoolean("VePID_b_PstnErrWithInDB : ", Robot.PID.getPID_b_PstnErrWithInDB());
    SmartDashboard.putNumber("VePID_Deg_PstnErrAccum :   ", Robot.PID.getPID_Deg_ErrAccum());
    SmartDashboard.putNumber("VePID_Pct_FdFwdCorr :      ", Robot.PID.getPIDRot_Pct_FdFwdTerm());
    SmartDashboard.putNumber("VePID_Pct_PropCorr :       ", Robot.PID.getPID_Pct_PropTerm());
    SmartDashboard.putNumber("VePID_Pct_IntglCorr :      ", Robot.PID.getPID_Pct_IntglTerm());
    SmartDashboard.putNumber("VePID_Pct_PwrCmnd :        ", Robot.PID.getPID_Pct_PIDCmndPct());
  }

  /* Called to Update RoboRIO Logfile for Display of Target Closed-Loop Calculations */
  public void updINS_RRL_PID_CorrCalc() { 
    System.out.println("VePID_Deg_PstnAct :       " + Robot.PID.getPID_Deg_PstnAct());
    System.out.println("VePID_Deg_PstnErr :       " + Robot.PID.getPID_Deg_PstnErr());
    System.out.println("VePID_b_PstnErrWithInDB : " + Robot.PID.getPID_b_PstnErrWithInDB());
    System.out.println("VePID_Deg_PstnErrAccum :  " + Robot.PID.getPID_Deg_ErrAccum());
    System.out.println("VePID_Pct_FdFwdCorr :     " + Robot.PID.getPIDRot_Pct_FdFwdTerm());
    System.out.println("VePID_Pct_PropCorr :      " + Robot.PID.getPID_Pct_PropTerm());
    System.out.println("VePID_Pct_IntglCorr :     " + Robot.PID.getPID_Pct_IntglTerm());
    System.out.println("VePID_Pct_PwrCmnd :       " + Robot.PID.getPID_Pct_PIDCmndPct());
  }
   


  /* Called to Update Smart Dashboard Data for Display of Drive Navigation System Data */
  public void updINS_SDB_NAV_Sys() { 
    SmartDashboard.putBoolean("VeNAV_b_CL_TgtRqstActv : ", Robot.NAV.getNAV_CL_TgtRqstActv());
    SmartDashboard.putBoolean("VeNAV_b_CL_DrvRqstActv : ", Robot.NAV.getNAV_CL_DrvRqstActv());
    SmartDashboard.putBoolean("VeNAV_b_DrvAutoCmdActv : ", Robot.NAV.getNAV_b_DrvAutoCmdActv());
    SmartDashboard.putBoolean("VeNAV_b_DrvStkRqstActv : ", Robot.NAV.getNAV_b_DrvStkRqstActv());
    SmartDashboard.putNumber("VeNAV_r_NormPwrCmdLong :  ", Robot.NAV.getNAV_r_NormPwrCmdLong());
    SmartDashboard.putNumber("VeNAV_r_NormPwrCmdLat :   ", Robot.NAV.getNAV_r_NormPwrCmdLat());
    SmartDashboard.putNumber("VeNAV_r_NormPwrCmdRot :   ", Robot.NAV.getNAV_r_NormPwrCmdRot());
    SmartDashboard.putNumber("VeNAV_r_NormPwrDrvrLtY :  ", Robot.NAV.getNAV_r_NormPwrDrvrLtY());
    SmartDashboard.putNumber("VeNAV_r_NormPwrDrvrLtX :  ", Robot.NAV.getNAV_r_NormPwrDrvrLtX());
    SmartDashboard.putNumber("VeNAV_r_NormPwrDrvrRtX :  ", Robot.NAV.getNAV_r_NormPwrDrvrRtX());
  }

  /* Called to Update RoboRIO Logfile for Display of Drive Navigation System Data */
  public void updINS_RRL_NAV_Sys() { 
    System.out.println("VeNAV_b_CL_TgtRqstActv : " + Robot.NAV.getNAV_CL_TgtRqstActv());
    System.out.println("VeNAV_b_CL_DrvRqstActv : " + Robot.NAV.getNAV_CL_DrvRqstActv());
    System.out.println("VeNAV_b_DrvAutoCmdActv : " + Robot.NAV.getNAV_b_DrvAutoCmdActv());
    System.out.println("VeNAV_b_DrvStkRqstActv : " + Robot.NAV.getNAV_b_DrvStkRqstActv());
    System.out.println("VeNAV_r_NormPwrCmdLong : " + Robot.NAV.getNAV_r_NormPwrCmdLong());
    System.out.println("VeNAV_r_NormPwrCmdLat :  " + Robot.NAV.getNAV_r_NormPwrCmdLat());
    System.out.println("VeNAV_r_NormPwrCmdRot :  " + Robot.NAV.getNAV_r_NormPwrCmdRot());
    System.out.println("VeNAV_r_NormPwrDrvrLtY : " + Robot.NAV.getNAV_r_NormPwrDrvrLtY());
    System.out.println("VeNAV_r_NormPwrDrvrLtX : " + Robot.NAV.getNAV_r_NormPwrDrvrLtX());
    System.out.println("VeNAV_r_NormPwrDrvrRtX : " + Robot.NAV.getNAV_r_NormPwrDrvrRtX());
  }


  /* Called to Update Smart Dashboard Data for Display of Lift System Data */
  public void updINS_SDB_Lift_Sys() { 
    SmartDashboard.putBoolean("VeLFT_b_JackExtdCmnd :   ", Robot.LIFT.getLFT_b_JackExtdCmnd());
    SmartDashboard.putBoolean("VeLFT_b_JackRtctCmnd :   ", Robot.LIFT.getLFT_b_JackRtctCmnd());
    SmartDashboard.putBoolean("VeLFT_b_DrwrExtdCmnd :   ", Robot.LIFT.getLFT_b_DrwrExtdCmnd());
    SmartDashboard.putBoolean("VeLFT_b_DrwrRtctCmnd :   ", Robot.LIFT.getLFT_b_DrwrRtctCmnd());
    SmartDashboard.putBoolean("VeLFT_b_StblzrExtdCmnd : ", Robot.LIFT.getLFT_b_StblzrExtdCmnd());
    SmartDashboard.putBoolean("VeLFT_b_StblzrRtctCmnd : ", Robot.LIFT.getLFT_b_StblzrRtctCmnd());
    SmartDashboard.putString("VeLFT_e_JackLckCmnd :    ", Robot.LIFT.getLFT_e_JackLckCmnd());
  }

  /* Called to Update RoboRIO Logfile for Display of Lift System Data */
  public void updINS_RRL_Lift_Sys() { 
    System.out.println("VeLFT_b_JackExtdCmnd :   " + Robot.LIFT.getLFT_b_JackExtdCmnd());
    System.out.println("VeLFT_b_JackRtctCmnd :   " + Robot.LIFT.getLFT_b_JackRtctCmnd());
    System.out.println("VeLFT_b_DrwrExtdCmnd :   " + Robot.LIFT.getLFT_b_DrwrExtdCmnd());
    System.out.println("VeLFT_b_DrwrRtctCmnd :   " + Robot.LIFT.getLFT_b_DrwrRtctCmnd());
    System.out.println("VeLFT_b_StblzrExtdCmnd : " + Robot.LIFT.getLFT_b_StblzrExtdCmnd());
    System.out.println("VeLFT_b_StblzrRtctCmnd : " + Robot.LIFT.getLFT_b_StblzrRtctCmnd());
    System.out.println("VeLFT_b_StblzrRtctCmnd : " + Robot.LIFT.getLFT_b_StblzrRtctCmnd());
    System.out.println("VeLFT_e_JackLckCmnd :   " + Robot.LIFT.getLFT_e_JackLckCmnd());
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
