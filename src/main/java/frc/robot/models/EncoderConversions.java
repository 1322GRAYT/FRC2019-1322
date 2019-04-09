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

  private static final int countsPerCycle = 1024 * 4; // 1024 ticks at 4x encoder (4x Encoder counts Up and Down)
  private static final double diaOfWheel = 4; // in inches
  private static final double circOfWheel = Math.PI * diaOfWheel; // D * PI
  private static final double gearRatio = (54 / 28) * (22 / 12); // 28 to 54 to 12 to 22

  public static int inToTicks(double Inches) {
    return (int) (Inches * (gearRatio * countsPerCycle / circOfWheel));
  }

  public static double ticksToIn(int Ticks) {
    return ((double) Ticks) / (gearRatio * countsPerCycle / circOfWheel);
  }
  

}