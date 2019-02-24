/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

/**
 * Class: K_Arm - Contains Control Calibrations for the Arm system.
 */
public class K_Arm {

    /**************************************************************/
	/*  Arm Position Encoder Counts Array Cell Index Assignments  */
	/**************************************************************/	 	 
    public static final int CeARM_BallPickUpFloor = 0;
    public static final int CeARM_HatchLow        = 1;
    public static final int CeARM_BallRocketLow   = 2;
    public static final int CeARM_BallPickUpFeed  = 3;
    public static final int CeARM_HatchMid        = 4;
    public static final int CeARM_BallCargoShip   = 5;
    public static final int CeARM_BallRocketMid   = 6;
    public static final int CeARM_HatchHigh       = 7;
    public static final int CeARM_BallRocketHigh  = 8;


    /********************************************/
	/*  Arm Position Encoder Count Definitions  */
	/********************************************/	 	
	
	/** KARM_Cnt_BallPickUpFloor: Arm Position in Encoder Counts associated
     *  with the Floor for Ball/Cargo pick-up. */
    public static final int KARM_Cnt_BallPickUpFloor = 0;  // Cell CeARM_BallPickUpFloor

 	/** KARM_Cnt_BallPickUpFeed: Arm Position in Encoder Counts associated
     *  with the Human Feed for Ball/Cargo pick-up. */
     public static final int KARM_Cnt_BallPickUpFeed = 173133;  // Cell CeARM_BallPickUpFeed

 	/** KARM_Cnt_BallCargoShip: Arm Position in Encoder Counts associated
     *  with the Cargo Ship Ball/Cargo placement. */
     public static final int KARM_Cnt_BallCargoShip = 197000;  // Cell CeARM_BallCargoShip

	/** KARM_Cnt_BallRocketLow: Arm Position in Encoder Counts associated
     *  with the Rocket LOW level Ball/Cargo placement. */
    public static final int KARM_Cnt_BallRocketLow = 114800;  // Cell CeARM_BallRocketLow
   
	/** KARM_Cnt_BallRocketMid: Arm Position in Encoder Counts associated
     *  with the Rocket MID level Ball/Cargo placement. */
    public static final int KARM_Cnt_BallRocketMid =  254185;  // Cell CeARM_BallRocketMid
    
	/** KARM_Cnt_BallRocketHigh: Arm Position in Encoder Counts associated
     *  with the Rocket HIGH level Ball/Cargo placement. */
    public static final int KARM_Cnt_BallRocketHigh = 402000;  // Cell CeARM_BallRocketHigh

	/** KARM_Cnt_HatchLow: Arm Position in Encoder Counts associated
     *  with the LOW level hatch placement. */
    public static final int KARM_Cnt_HatchLow = 24312;  // Cell CeARM_HatchLow

	/** KARM_Cnt_HatchMid: Arm Position in Encoder Counts associated
     *  with the MID level hatch placement. */
    public static final int KARM_Cnt_HatchMid = 185000;  // Cell CeARM_HatchMid

	/** KARM_Cnt_HatchHigh: Arm Position in Encoder Counts associated
     *  with the HIGH level hatch placement. */
    public static final int KARM_Cnt_HatchHigh = 312056;  // Cell CeARM_HatchHigh
    
}
