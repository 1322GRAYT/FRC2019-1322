/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import frc.robot.models.PositionData;
import frc.robot.models.GamePieces;

/*******************************************************************
 * Class: K_Arm - Contains Control Calibrations for the Arm system.
 *******************************************************************/
public class K_Arm {

    /***********************************************************
     * Button timeout used in both the Arm Hold Buttons
     ***********************************************************/
    public static final double BUTTON_TIMEOUT = 0.5;

    /***********************************************************
     * This is the tolerance for all the arm position commands
     ***********************************************************/
    public final static int TOLERANCE = 1000;

    /***********************************************************
     * Arm Position Encoder Count Definitions
     ***********************************************************/
    public static final List<PositionData> ALL_POS_DATA = Arrays.asList(
                    new PositionData(0, "Floor Pickup", GamePieces.Cargo),
                    new PositionData(8307, "Rocket Level 1", GamePieces.HatchPanel),
                    new PositionData(114800, "Rocket Level 1", GamePieces.Cargo),
                    // new PositionData(173133, "Human Feed", GamePieces.Cargo),
                    new PositionData(172700, "Rocket Level 2", GamePieces.HatchPanel),
                    new PositionData(197000, "Cargo Ship", GamePieces.Cargo),
                    new PositionData(254185, "Rocket Level 2", GamePieces.Cargo),
                    new PositionData(287900, "Rocket Level 3", GamePieces.HatchPanel),
                    new PositionData(402000, "Rocket Level 3", GamePieces.Cargo));

    public static final List<PositionData> BALL_POS_DATA = ALL_POS_DATA.stream()
                    .filter(p -> p.gpType == GamePieces.Cargo || p.gpType == GamePieces.Both)
                    .collect(Collectors.toList());
    public static final List<PositionData> PANEL_POS_DATA = ALL_POS_DATA.stream()
                    .filter(p -> p.gpType == GamePieces.HatchPanel || p.gpType == GamePieces.Both)
                    .collect(Collectors.toList());




    /**************************************************/
    /*  Arm System Feed-Forward Term                */
 	/**************************************************/	 	

	/** KARM_dPct_VelFeedFwdTerm: Arm System Velocity Control
     * Feed Forward Term for Motion Magic Control in units of
     * 1023 * duty-cycle / encoder-counter-per-100ms.
     */
    public static final double KARM_dPct_VelFeedFwdTerm = 0.11;


    /**************************************************/
    /*  Arm System PID Gains                       */
 	/**************************************************/	 	

	/** KARM_k_VelPropGx: Arm System Velocity Control
     * Proportional Gain.
     */
    public static final double KARM_k_VelPropGx = 0.13;
     
	/** KARM_k_VelIntglGx: Arm System Velocity Control
     * Integral Gain.
     */
    public static final double KARM_k_VelIntglGx = 0.00007;

	/** KARM_k_VelDerivGx: Arm System Velocity Control
     * Derivative Gain.
     */
    public static final double KARM_k_VelDerivGx = 0.0013;

   
    /**************************************************/
    /*  Arm System Motion Speed Profile Settings    */
    /**************************************************/	 	

	/** KARM_n_MM_CruiseVel: Arm System Motion Magic
     * Speed Profiling Cruise Velocity in units of encoder
     * counts per 100ms.
     */
    public static final int KARM_n_MM_CruiseVel = 11000;
     
	/** KARM_a_MM_MaxAccel: Arm System Motion Magic
     * Speed Profiling Maximum Acceleration in units of
     * encoder counts per 100ms per sec.
     */
    public static final int KARM_a_MM_MaxAccel = 12000;

}
