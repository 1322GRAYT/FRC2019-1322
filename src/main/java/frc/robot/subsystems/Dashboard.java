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
