/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.*;
import frc.robot.calibrations.K_System;


public class CC_CamCaptureTgt extends Command {
  public CC_CamCaptureTgt() {
     requires(Robot.VISION);
  }
  
  boolean LeVSN_b_ImgCaptureVld;


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    LeVSN_b_ImgCaptureVld = false;
    Robot.NAV.setNAV_CL_TgtRqstActv(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // Camera: Target Image Capture - Testing
	  if (K_System.KeSYS_e_DebugEnblVsn != DebugSlct.DebugDsbl) {
      System.out.println("Cam Capture Target Begin Execute. ");
      LeVSN_b_ImgCaptureVld = Robot.VISION.dtrmnVSN_CamVldData();
      System.out.println("Cam Capture Target Complete Execute. ");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean CmndCmpt = false;

    if (K_System.KeSYS_e_DebugEnblVsn != DebugSlct.DebugDsbl) {
      CmndCmpt = LeVSN_b_ImgCaptureVld; 
    }
    else {
      CmndCmpt = true;
    }

    return CmndCmpt;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.DASHBOARD.updINS_SDB_PID_CL_Err();
    Robot.DASHBOARD.updINS_RRL_PID_CL_Err();
    Robot.DASHBOARD.updINS_SDB_PID_CorrCalc();
    Robot.DASHBOARD.updINS_RRL_PID_CorrCalc();
    System.out.println("Cam Capture Target Begin Processing. ");

    if (K_System.KeSYS_e_DebugEnblVsn != DebugSlct.DebugDsbl) {  
      Robot.VISION.MngVSN_CamImgProc();
    }

    if ((K_System.KeSYS_e_DebugEnblVsn == DebugSlct.DebugEnblBoth) ||
        (K_System.KeSYS_e_DebugEnblVsn == DebugSlct.DebugEnblSDB)) {
      Robot.DASHBOARD.updINS_SDB_VSN_CamImgCalc();
    }
    if ((K_System.KeSYS_e_DebugEnblVsn == DebugSlct.DebugEnblBoth) ||
        (K_System.KeSYS_e_DebugEnblVsn == DebugSlct.DebugEnblRRL)) {
      Robot.DASHBOARD.updINS_RRL_VSN_CamImgCalc();
    }

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
