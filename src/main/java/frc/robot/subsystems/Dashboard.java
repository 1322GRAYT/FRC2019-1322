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

  public enum DebugSlct {
    DebugDsbl, DebugEnblSDB, DebugEnblRRL, DebugEnblBoth
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


  /* Called to Update Smart Dashboard Data for Display of Drive Navigation System Data */
  public void updINS_SDB_NAV_Sys() { 
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
