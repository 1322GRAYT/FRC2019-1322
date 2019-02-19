/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* The I2C color codes are as follows:
'r' = Set to red, 'g' = Set to green, 'b' = Set to blue, 'u' = Rainbow, 'c' = Rainbow cycle,
'h' = Chase, 'o' = Off
'r' = 114, 'g' = 103, 'b' = 98, 'u' = 117, 'c' = 99, 'h' = 104, 'o' = 111, 't' = 116, 'p' = 112, 's' = 115
'o' -> 'r' -> 'g' -> 'b' -> 'u' -> 'c' -> 'h' -> 't' -> 'p' -> 's' -> ...
*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class LEDController extends Subsystem {
  
 private I2C ledArduino;

 // LED Colors:
 public int ledRed = 114;
 public int ledGreen = 103;
 public int ledBlue = 98;
 public int ledYellow = 121;
 public int ledRainbow = 117;
 public int ledRainbowCycle = 99;
 public int ledVisualize = 115;
 public int ledOff = 111;
 //LED Modes
 public int ledModeChase = 104;
 public int ledModeCheckerboard = 116;
 public int ledModeBreath = 112;

 public LEDController() {
   ledArduino = new I2C(I2C.Port.kOnboard, 0x08);
 }

 public void setLEDs(int color){
    ledArduino.write(0x08, color);
 }

 public void setMode(int color, int mode){
    ledArduino.write(0x08, mode);
    ledArduino.write(0x08, color);
 }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new CC_LEDCntrl());
  }
}
