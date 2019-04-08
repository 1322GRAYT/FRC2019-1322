/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.calibrations.K_Lift;
import frc.robot.Robot;

/**
 * Subsystem for the Lift system which controls the Scissor Lift Mechanism, the
 * Drawer-Slide translate mechanism, and the Hydraulic Stabilizer wheels.
 */
public class Lift extends Subsystem {

  private boolean VeLFT_b_JackExtdRqst;
  private boolean VeLFT_b_JackRtctRqst;
  private boolean VeLFT_b_DrwrExtdRqst;
  private boolean VeLFT_b_DrwrRtctRqst;
  private boolean VeLFT_b_StblzrExtdRqst;
  private boolean VeLFT_b_StblzrRtctRqst;


  /*******************************/
  /* Public Class Methods        */
  /*******************************/

  public void mngLFT_InitCntrl() {
    rstLFT_InpRqstFlgs();
  }

  public void mngLFT_CntrlSys() {
    cntrlLFT_System();
    rstLFT_InpRqstFlgs();  /* Clear Request flags before OI which runs early next loop */
  }

  /*******************************/
  /* Public Class Interfaces     */
  /*******************************/

  public void setLFT_JackExtdRqst(boolean state) {
    VeLFT_b_JackExtdRqst = state;
  }

  public void setLFT_JackRtctRqst(boolean state) {
    VeLFT_b_JackRtctRqst = state;
  }

  public void setLFT_DrwrExtdRqst(boolean state) {
    VeLFT_b_DrwrExtdRqst = state;
  }

  public void setLFT_DrwrRtctRqst(boolean state) {
    VeLFT_b_DrwrRtctRqst = state;
  }

  public void setLFT_StblzrExtdRqst(boolean state) {
    VeLFT_b_StblzrExtdRqst = state;
  }

  public void setLFT_StblzrRtctRqst(boolean state) {
    VeLFT_b_StblzrRtctRqst = state;
  }



  /*******************************/
  /* Private Class Methods       */
  /*******************************/

  private void cntrlLFT_System() {
    double NormPwrVert, NormPwrHorz;
    boolean DrwrMvmtRvrs = false;
    boolean StblzrExtd = false; 

    /* Lift Scissor Vertical Control */
    if (VeLFT_b_JackExtdRqst == true) {
      NormPwrVert = K_Lift.KeLFT_r_NormPwrExtdVert;
    } 
    else if (VeLFT_b_JackRtctRqst == true) {
      NormPwrVert = -K_Lift.KeLFT_r_NormPwrRtctVert;
    } 
    else {
      NormPwrVert = 0.0;       
    }
    Robot.SCISSOR.liftRobot(NormPwrVert);       


    /* Lift Slide Horizontal Control */
    if (VeLFT_b_DrwrExtdRqst == true) {
      NormPwrHorz = K_Lift.KeLFT_r_NormPwrExtdHorz;
    } else if (VeLFT_b_DrwrRtctRqst == true) {
      NormPwrHorz = -K_Lift.KeLFT_r_NormPwrRtctHorz;
      DrwrMvmtRvrs = true;
    } else {
      NormPwrHorz = 0.0;
    }
    Robot.SCISSOR.extendLift(NormPwrHorz);


    /* Lift Stabalizer Wheel Control */
    if ((DrwrMvmtRvrs == true) && (Robot.SCISSOR.getFloorSensor() == false)) {
      /* If Drawer Slide is moving backwards and limit switch is detected, retract stabalizer wheels */
      StblzrExtd = false;
    } 
    else if (VeLFT_b_StblzrExtdRqst == true) {
      StblzrExtd = true;
    }
    else if (VeLFT_b_StblzrRtctRqst == true) {
      if (Robot.SCISSOR.getLimits()) {
        /* Override Retract Request with Extend because Limit Switch is indicated */
        StblzrExtd = true;
      } 
      else {
        StblzrExtd = false;
      }
    } 
    else {
      StblzrExtd = false;
    }
  Robot.SCISSOR.liftRobotPnumatic(StblzrExtd);

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
