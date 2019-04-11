/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.calibrations.K_System;
import frc.robot.calibrations.K_Lift;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Subsystem for the Lift system which controls the Scissor Lift Mechanism, the
 * Drawer-Slide translate mechanism, and the Hydraulic Stabilizer wheels.
 */
public class Lift extends Subsystem {

  private Relay VeLFT_b_JackLckApply;

  private boolean VeLFT_b_JackExtdRqst;
  private boolean VeLFT_b_JackRtctRqst;
  private boolean VeLFT_b_DrwrExtdRqst;
  private boolean VeLFT_b_DrwrRtctRqst;
  private boolean VeLFT_b_StblzrExtdRqst;
  private boolean VeLFT_b_StblzrRtctRqst;

  private boolean VeLFT_b_StblzrExtdCmnd;

  public Lift() {
    VeLFT_b_JackLckApply = new Relay(RobotMap.RobotScissorLock);
  }


  /*******************************/
  /* Public Class Methods        */
  /*******************************/

  public void mngLFT_InitCntrl() {
    rstLFT_InpRqstFlgs();
    setLFT_b_StblzrExtdCmnd(false);
  }

  public void mngLFT_CntrlSys() {
    cntrlLFT_System();
    rstLFT_InpRqstFlgs();  /* Clear Request flags before OI which runs early next loop */
  }

  /*******************************/
  /* Public Class Interfaces     */
  /*******************************/

  public void setLFT_b_JackExtdRqst(boolean state) {
    VeLFT_b_JackExtdRqst = state;
  }

  public void setLFT_b_JackRtctRqst(boolean state) {
    VeLFT_b_JackRtctRqst = state;
  }

  public void setLFT_b_DrwrExtdRqst(boolean state) {
    VeLFT_b_DrwrExtdRqst = state;
  }

  public void setLFT_b_DrwrRtctRqst(boolean state) {
    VeLFT_b_DrwrRtctRqst = state;
  }

  public void setLFT_b_StblzrExtdRqst(boolean state) {
    VeLFT_b_StblzrExtdRqst = state;
  }

  public void setLFT_b_StblzrRtctRqst(boolean state) {
    VeLFT_b_StblzrRtctRqst = state;
  }

  public void setLFT_b_StblzrExtdCmnd(boolean state) {
    VeLFT_b_StblzrExtdCmnd = state;
  }


  public boolean getLFT_b_JackExtdRqst() {
    return(VeLFT_b_JackExtdRqst);
  }

  public boolean getLFT_b_JackRtctRqst() {
    return(VeLFT_b_JackRtctRqst);
  }

  public boolean getLFT_b_DrwrExtdRqst() {
    return(VeLFT_b_DrwrExtdRqst);
  }

  public boolean getLFT_b_DrwrRtctRqst() {
    return(VeLFT_b_DrwrRtctRqst);
  }

  public boolean getLFT_b_StblzrExtdRqst() {
    return(VeLFT_b_StblzrExtdRqst);
  }

  public boolean getLFT_b_StblzrRtctRqst() {
    return(VeLFT_b_StblzrRtctRqst);
  }

  public boolean getLFT_b_StblzrExtdCmnd() {
    return(VeLFT_b_StblzrExtdCmnd);
  }
 
  public String getLFT_b_JackLckApply() {
    return(VeLFT_b_JackLckApply.get().toString());
  }


  /*******************************/
  /* Private Class Methods       */
  /*******************************/

  private void cntrlLFT_System() {
    double NormPwrVert, NormPwrHorz;
    boolean JackLck;
    boolean DrwrMvmtFwd = false;
    boolean StblzrExtd = false; 


    /* Lift Scissor Vertical Control */
    if (VeLFT_b_JackExtdRqst == true) {
      NormPwrVert = K_Lift.KeLFT_r_NormPwrExtdVert;
      JackLck = false;
    } 
    else if (VeLFT_b_JackRtctRqst == true) {
      NormPwrVert = -K_Lift.KeLFT_r_NormPwrRtctVert;
      JackLck = false;
    } 
    else {
      NormPwrVert = 0.0;
      JackLck = true;
    }
    Robot.SCISSOR.liftRobot(NormPwrVert);       
    VeLFT_b_JackLckApply.set((JackLck) ? Relay.Value.kForward : Relay.Value.kOff);

    /* Lift Slide Horizontal Control */
    if (VeLFT_b_DrwrExtdRqst == true) {
      NormPwrHorz = -K_Lift.KeLFT_r_NormPwrExtdHorz;
      DrwrMvmtFwd = true;
    } else if (VeLFT_b_DrwrRtctRqst == true) {
      NormPwrHorz = K_Lift.KeLFT_r_NormPwrRtctHorz;
    } else {
      NormPwrHorz = 0.0;
    }
    Robot.SCISSOR.extendLift(NormPwrHorz);


    /* Lift Stabalizer Wheel Control */

    StblzrExtd = VeLFT_b_StblzrExtdCmnd;

    if ((DrwrMvmtFwd == true) && (Robot.SCISSOR.getFloorSensor() == false)) {
      /* If Drawer Slide is moving forwards and limit switch is detected, auto-extend stabalizer wheels */
      StblzrExtd = true;
    } 
    else if (VeLFT_b_StblzrExtdRqst == true) {
      StblzrExtd = true;
    }
    else if (VeLFT_b_StblzrRtctRqst == true) {
      StblzrExtd = false;    
    } 
    else {
       /* No Update */
    }
    VeLFT_b_StblzrExtdCmnd = StblzrExtd;
   Robot.SCISSOR.liftRobotPnumatic(!VeLFT_b_StblzrExtdCmnd);
   if (K_System.KeSYS_b_DebugEnblLft) {
     Robot.DASHBOARD.updINS_SDB_Lift_Sys();
     Robot.DASHBOARD.updINS_RRL_Lift_Sys();
   }

  }


  private void rstLFT_InpRqstFlgs() {
    VeLFT_b_JackExtdRqst = false;
    VeLFT_b_JackRtctRqst = false;
    VeLFT_b_DrwrExtdRqst = false;
    VeLFT_b_DrwrRtctRqst = false;
    VeLFT_b_StblzrExtdRqst = false;
    VeLFT_b_StblzrRtctRqst = false;
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
