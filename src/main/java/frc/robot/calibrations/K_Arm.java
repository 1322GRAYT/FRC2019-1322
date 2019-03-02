/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

import frc.robot.models.PositionData;

/**
 * Class: K_Arm - Contains Control Calibrations for the Arm system.
 */
public class K_Arm {

    /********************************************/
	/*  Arm Position Encoder Count Definitions  */
	/********************************************/	 	
    // As a list, this is complicated to use but most precice
    // The idea is to use this as a stream and filter its use.
    /*
    public static List<PositionData> L_ARM_POS_DATA = Arrays.asList(
        new PositionData(0, "Floor Pickup", "Ball"),
        new PositionData(24312, "Rocket Level 1", "Panel"),
        new PositionData(114800, "Rocket Level 1", "Ball"),
        new PositionData(173133, "Human Feed", "Ball"),
        new PositionData(185000, "Rocket Level 2", "Panel"),
        new PositionData(197000, "Cargo Ship", "Ball"),
        new PositionData(254185, "Rocket Level 2", "Ball"),
        new PositionData(312056, "Rocket Level 3", "Panel"),
        new PositionData(402000, "Rocket Level 3", "Ball")
    );
    int[] test = L_ARM_POS_DATA.stream().filter(p -> p.name == "Ball" || p.name == "Both"). */

    public final static PositionData[] ARM_POS_DATA = {
        new PositionData(0, "Floor Pickup", "Ball"),
        new PositionData(20500, "Rocket Level 1", "Panel"),
        new PositionData(98000, "Rocket Level 1", "Ball"),
        new PositionData(154500, "Rocket Level 2 / Cargo Ship", "Panel"),
        new PositionData(229000, "Rocket Level 2", "Ball"),
        new PositionData(297000, "Rocket Level 3", "Panel"),
        new PositionData(388000, "Rocket Level 3", "Ball")
    };
    public final static int MAX_ARM_POSITION = ARM_POS_DATA.length - 1;

    // Define Ball Positions - Max is used to ensure no overflow errors
    public final static int[] BALL_POSITIONS = {0, 2, 4, 6};
    public final static int MAX_BALL_POSITION = BALL_POSITIONS.length - 1;

    // Define Panel Positions - Max is used to ensure no overflow errors
    public final static int[] PANEL_POSITIONS = {1, 3, 5};
    public final static int MAX_PANEL_POSITION = PANEL_POSITIONS.length - 1;

    // This is the tolerance for all the arm position commands:
    public final static int TOLERANCE = 1000;
    
}
