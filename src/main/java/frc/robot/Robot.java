/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.calibrations.K_System;
import frc.robot.commands.*;
import frc.robot.models.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;

  public final static TblLib          TBLLIB    = new TblLib();
  public final static Vision          VISION    = new Vision();
  public final static CLpid           PID       = new CLpid();
  public final static Drives          DRIVES    = new Drives();
  public final static Nav             NAV       = new Nav();
  public final static Claw            CLAW      = new Claw();
  public final static LEDController   LEDS      = new LEDController();
  public final static Dashboard       DASHBOARD = new Dashboard();
  public final static Arm             ARM       = new Arm();
  public final static Lift            LIFT      = new Lift();
  public final static Scissor         SCISSOR   = new Scissor();

  // Cameras
  public static UsbCamera camera0;
  public static UsbCamera camera1;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();

    // Camera Server
    camera0 = CameraServer.getInstance().startAutomaticCapture(0);
    camera1 = CameraServer.getInstance().startAutomaticCapture(1);
    // Set Keep Connections Open to Keep Switching Speed Fast!
    camera0.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    VISION.mngVSN_InitLimeLightNetTbl(); 
    VISION.mngVSN_InitCamCalibr();
    PID.setPID_Deg_PstnTgt(false, 0.0);
    PID.mngPID_InitCntrl();
    NAV.mngNAV_InitCntrl();
    LIFT.mngLFT_InitCntrl();

    m_chooser.setDefaultOption("Driver Control Only", new CA_RevertToTele());
		m_chooser.addOption("Auto Drive Forward", new CA_DrvPstnTgt(0, 140000));
    SmartDashboard.putData("Auto Mode:", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    VISION.mngVSN_CamImgPeriodic();
    NAV.mngNAV_CmndSysTsk1();
    PID.mngPID_Cntrl(); 
    NAV.mngNAV_CmndSysTsk2();
    if (K_System.KeSYS_b_NewLiftEnbl == true) {
      LIFT.mngLFT_CntrlSys();
    }
  }


  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    System.out.println("teleopPeriodic *** ");
    Scheduler.getInstance().run();
  } 


  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }


  public static double deadzonify(double number) {
    if(Math.abs(number) > .07) {
      return number;
    }
    return 0;
  }

}
