/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Dashboard.*;
import frc.robot.subsystems.LEDController.liftLED;
import frc.robot.calibrations.K_System;
import frc.robot.calibrations.K_Lift;


/**
 * Subsystem for the Lift system which controls the Scissor Lift Mechanism, the
 * Drawer-Slide translate mechanism, and the Hydraulic Stabilizer wheels.
 */
public class Lift extends Subsystem {

  public enum ActuatorSt {
    Hold, Extend, Retract
  }

  /* Driver or Autonomous Request */  
  private boolean VeLFT_b_JackExtdRqst;
  private boolean VeLFT_b_JackRtctRqst;
  private boolean VeLFT_b_DrwrExtdRqst;
  private boolean VeLFT_b_DrwrRtctRqst;
  private boolean VeLFT_b_StblzrExtdRqst;
  private boolean VeLFT_b_StblzrRtctRqst;

  /* Command to Motors */
  private boolean VeLFT_b_JackExtdCmnd;
  private boolean VeLFT_b_JackRtctCmnd;
  private boolean VeLFT_b_DrwrExtdCmnd;
  private boolean VeLFT_b_DrwrRtctCmnd;
  private boolean VeLFT_b_StblzrExtdCmnd;
  private boolean VeLFT_b_StblzrRtctCmnd;
  private Relay VeLFT_e_JackLckCmnd;

  public Lift() {
    VeLFT_e_JackLckCmnd = new Relay(RobotMap.RobotScissorLock);
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

  /*** Set Functions ***/  
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

  public void setLFT_b_JackExtdCmnd(boolean state) {
    VeLFT_b_JackExtdCmnd = state;
  }

  public void setLFT_b_JackRtctCmnd(boolean state) {
    VeLFT_b_JackRtctCmnd = state;
  }

  public void setLFT_b_DrwrExtdCmnd(boolean state) {
    VeLFT_b_DrwrExtdCmnd = state;
  }

  public void setLFT_b_DrwrRtctCmnd(boolean state) {
    VeLFT_b_DrwrRtctCmnd = state;
  }

  public void setLFT_b_StblzrExtdCmnd(boolean state) {
    VeLFT_b_StblzrExtdCmnd = state;
  }

  public void setLFT_b_StblzrRtctCmnd(boolean state) {
    VeLFT_b_StblzrRtctCmnd = state;
  }

  public void setLFT_e_JackLckCmnd(Relay.Value state) {
    VeLFT_e_JackLckCmnd.set(state);
  }



  /*** Get Functions ***/
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

  public boolean getLFT_b_JackExtdCmnd() {
    return(VeLFT_b_JackExtdCmnd);
  }

  public boolean getLFT_b_JackRtctCmnd() {
    return(VeLFT_b_JackRtctCmnd);
  }
 
  public boolean getLFT_b_DrwrExtdCmnd() {
    return(VeLFT_b_DrwrExtdCmnd);
  }

  public boolean getLFT_b_DrwrRtctCmnd() {
    return(VeLFT_b_DrwrRtctCmnd);
  }

  public boolean getLFT_b_StblzrExtdCmnd() {
    return(VeLFT_b_StblzrExtdCmnd);
  }

  public boolean getLFT_b_StblzrRtctCmnd() {
    return(VeLFT_b_StblzrRtctCmnd);
  }

  public String getLFT_e_JackLckCmnd() {
    return(VeLFT_e_JackLckCmnd.get().toString());
  }


  /*******************************/
  /* Private Class Methods       */
  /*******************************/

  private void cntrlLFT_System() {
    double NormPwrVert, NormPwrHorz;
    boolean JackExtd, JackRtct, DrwrExtd, DrwrRtct, StblzrExtd, StblzrRtct; 
    Relay.Value JackLck;
    boolean DrwrMvmtFwd = false;


    /* Lift Scissor Vertical Control */
    if (VeLFT_b_JackExtdRqst == true) {
      JackExtd = true;
      JackRtct = false;
      NormPwrVert = K_Lift.KeLFT_r_NormPwrExtdVert;
      JackLck = Relay.Value.kOff;
    } 
    else if (VeLFT_b_JackRtctRqst == true) {
      JackExtd = false;
      JackRtct = true;
      NormPwrVert = -K_Lift.KeLFT_r_NormPwrRtctVert;
      JackLck = Relay.Value.kOff;
    } 
    else {
      JackExtd = false;
      JackRtct = false;
      NormPwrVert = 0.0;
      JackLck = Relay.Value.kForward;
    }
    setLFT_b_JackExtdCmnd(JackExtd);
    setLFT_b_JackRtctCmnd(JackRtct);
    Robot.SCISSOR.liftRobot(NormPwrVert);       
    setLFT_e_JackLckCmnd(JackLck);


    /* Lift Slide Horizontal Control */
    if (VeLFT_b_DrwrExtdRqst == true) {
      DrwrExtd = true;
      DrwrRtct = false;
      NormPwrHorz = -K_Lift.KeLFT_r_NormPwrExtdHorz;
      DrwrMvmtFwd = true;
    } else if (VeLFT_b_DrwrRtctRqst == true) {
      DrwrExtd = false;
      DrwrRtct = true;
      NormPwrHorz = K_Lift.KeLFT_r_NormPwrRtctHorz;
    } else {
      DrwrExtd = false;
      DrwrRtct = false;
      NormPwrHorz = 0.0;
    }
    setLFT_b_DrwrExtdCmnd(DrwrExtd);
    setLFT_b_DrwrRtctCmnd(DrwrRtct);
    Robot.SCISSOR.extendLift(NormPwrHorz); 


    /* Lift Stabalizer Wheel Control */
    if ((DrwrMvmtFwd == true) && (Robot.SCISSOR.getFloorSensor() == false)) {
      /* If Drawer Slide is moving forwards and limit switch is detected, auto-extend stabalizer wheels */
      StblzrExtd = true;
      StblzrRtct = false;
    } 
    else if (VeLFT_b_StblzrExtdRqst == true) {
      StblzrExtd = true;
      StblzrRtct = false;
    }
    else if (VeLFT_b_StblzrRtctRqst == true) {
      StblzrExtd = false;
      StblzrRtct = true;    
    } 
    else {
      /* Latch Previous Values */ 
      StblzrExtd = getLFT_b_StblzrExtdCmnd();
      StblzrRtct = getLFT_b_StblzrRtctCmnd();
    }
    setLFT_b_StblzrExtdCmnd(StblzrExtd);
    setLFT_b_StblzrRtctCmnd(StblzrRtct);
    Robot.SCISSOR.liftRobotPnumatic(getLFT_b_StblzrRtctCmnd());


    /* Control LED Array Indicators */
    if(getLFT_b_StblzrExtdCmnd() || getLFT_b_StblzrRtctCmnd()) {
      Robot.LEDS.setLED_ActnLift(liftLED.Stblzr);  
    }
    else if (getLFT_b_DrwrExtdCmnd() || getLFT_b_DrwrRtctCmnd()) {
      Robot.LEDS.setLED_ActnLift(liftLED.Drwr);
    }
    else if (getLFT_b_JackExtdCmnd() || getLFT_b_JackRtctCmnd()) {
      Robot.LEDS.setLED_ActnLift(liftLED.Jack);
    }
    else {
      Robot.LEDS.setLED_ActnLift(liftLED.InActv);
    }


    /* Update Instrumentation */
    if ((K_System.KeSYS_e_DebugEnblLft == DebugSlct.DebugEnblBoth) ||
        (K_System.KeSYS_e_DebugEnblLft == DebugSlct.DebugEnblSDB)) {
      Robot.DASHBOARD.updINS_SDB_Lift_Sys();
    }
    if ((K_System.KeSYS_e_DebugEnblLft == DebugSlct.DebugEnblBoth) ||
        (K_System.KeSYS_e_DebugEnblLft == DebugSlct.DebugEnblRRL)) {
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
