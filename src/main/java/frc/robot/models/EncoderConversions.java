package frc.robot.models;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.calibrations.K_Drive;


public class EncoderConversions {
    /************
   * Unit Conversions
   * 
   * inToTicks converts linear inches to encoder ticks For use to read encoder
   * counts to ticksToIn converts encoder ticks to linear inches
   */

  private static final int countsPerCycle = 1024 * 4; // 200 ticks at 4x encoder (4x Encoder counts Up and Down)
  private static final double gearRatio = (54 / 28) * (22 / 12); // 28 to 54 to 12 to 22 - Practice Bot

  public static int inToTicks(double Inches) {
    return (int) (Inches * (K_Drive.KDRV_r_EncdrToWhl * countsPerCycle / K_Drive.KDRV_l_DistPerRevWhl));
  }

  public static double ticksToIn(int Ticks) {
    return ((double) Ticks) / K_Drive.KDRV_r_EncdrToWhl* countsPerCycle / K_Drive.KDRV_l_DistPerRevWhl;
  }
  

  /** Method: cvrtAngToLinSpd - Calculates the Linear 
   *  Speed of the Robot from the Angular Wheel Speed
   *  when moving directly forward or rearward.
   *  @param: Wheel Angular Speed (rpm)
   *  @return: Robot Linear Speed (inch/sec) */	
  public static float cvrtAngToLinSpd(float SpdWhl) {
    return ((float)((K_Drive.KDRV_l_DistPerRevWhl * SpdWhl)/(float)60));
  }
  

  /** Method: cvrtDistToCnts - Calculates the nominal number 
   *  of Drive encoder counts (cnts) that would be registered if
   *  the the Drive Wheel traveled forward/backward the
   *  desired distance given (inches).
   *  @param: Desired Distance (inches)
   *  @return: Encoder Counts (cnts) */
  public static double cvrtDistToCnts(float DistInch) {
    double WhlRevs;
    double EncdrRevs;
    double EncdrCnts;
   
    WhlRevs   = (double)(DistInch / K_Drive.KDRV_l_DistPerRevWhl);	 
    EncdrRevs = WhlRevs * (double)K_Drive.KDRV_r_EncdrToWhl;	 
    EncdrCnts = EncdrRevs * (double)K_Drive.KDRV_Cnt_PlsPerRevEncdr;
   
    return EncdrCnts;
  }


  /** Method: cvrtCntsToDist - Calculates the nominal distance
   *  that would be/was travelled in inches based on the number
   *  of encoder counts that were registered by the the Drive Wheel
   *  encoder (cnts).
   *  @param: Encoder Counts (cnts) 
   *  @return: Desired Distance (inches) */
  public static double cvrtCntsToDist(int EncdrCnts) {
    double EncdrRevs;
    double WhlRevs;
    double DistInch;

    EncdrRevs = (double)EncdrCnts / (double)K_Drive.KDRV_Cnt_PlsPerRevEncdr;
    WhlRevs   = EncdrRevs / (double)K_Drive.KDRV_r_EncdrToWhl;	 
    DistInch  = WhlRevs * (double)K_Drive.KDRV_l_DistPerRevWhl;	 
  
    return DistInch;
  }


}