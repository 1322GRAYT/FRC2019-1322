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

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class LEDController extends Subsystem {

  public enum liftLED {
    InActv, Jack, Drwr, Stblzr
  }

  public enum clawLED {
    InActv, ClawExtd, ClawRtct, PanelEjct, PanelLtch
  }

  public enum colorLED {
    Off, Red, Green, Blue, Yellow, Rainbow, RainbowCycle, Visualize
  }

  public enum modeLED {
    Null, Chase, Checkerboard, Breath
  }
  
  private I2C ledArduino;

  private boolean  VeLED_b_CmndActv;
  private liftLED  VeLED_e_ActnLift;
  private clawLED  VeLED_e_ActnClaw;
  private colorLED VeLED_e_Color;
  private modeLED  VeLED_e_Mode; 
  private int      VeLED_Cnt_CodeColor;
  private int      VeLED_Cnt_CodeMode; 


  // LED Colors:
  private int ledRed = 114;
  private int ledGreen = 103;
  private int ledBlue = 98;
  private int ledYellow = 121;
  private int ledRainbow = 117;
  private int ledRainbowCycle = 99;
  private int ledVisualize = 115;
  private int ledOff = 111;
  //LED Modes
  private int ledModeChase = 104;
  private int ledModeCheckerboard = 116;
  private int ledModeBreath = 112;
  private int ledNull       = -1;


  public LEDController() {
    ledArduino = new I2C(I2C.Port.kOnboard, 0x08);
  }



  /*******************************/
  /* Public Class Interfaces     */
  /*******************************/

  public void setLED_b_CmndActv(boolean actvCmnd){
    VeLED_b_CmndActv = actvCmnd;
  }

  public void setLED_ActnLift(liftLED actnLIFT){
    VeLED_e_ActnLift = actnLIFT;
  }

  public void setLED_ActnClaw(clawLED actnClaw){
    VeLED_e_ActnClaw = actnClaw;
  }

  public void reqLED_Color(colorLED colorReq){
    VeLED_e_Color = colorReq;
  }

  public void reqLED_Mode(modeLED modeReq){
    VeLED_e_Mode = modeReq;
  }



  /*******************************/
  /* Public Class Methods        */
  /*******************************/


  public void mngLED_InitCntrl(){
    /* */
  }

  public void mngLED_CntrlSys(){
    CntrlLED_ArbCmnd();
  }



  /*******************************/
  /* Private Class Methods       */
  /*******************************/

  private void CntrlLED_ArbCmnd(){
    colorLED LeLED_e_Color;
    modeLED  LeLED_e_Mode; 
    int LeLED_Cnt_CodeColor;
    int LeLED_Cnt_CodeMode;

    /*******************************/
    /* Arbitrate Color and Mode    */
    /*******************************/
    if (VeLED_b_CmndActv == true) {
      LeLED_e_Color = VeLED_e_Color;
      LeLED_e_Mode =  VeLED_e_Mode;
    }
    else if (VeLED_e_ActnLift == liftLED.Stblzr)  {
      LeLED_e_Color = colorLED.Rainbow;
      LeLED_e_Mode =  modeLED.Null;
    }
    else if (VeLED_e_ActnLift == liftLED.Drwr)  {
      LeLED_e_Color = colorLED.Yellow;
      LeLED_e_Mode =  modeLED.Chase;
    }
    else if (VeLED_e_ActnLift == liftLED.Jack)  {
      LeLED_e_Color = colorLED.Red;
      LeLED_e_Mode =  modeLED.Chase;
    }
    else if (VeLED_e_ActnClaw == clawLED.PanelEjct) {
      LeLED_e_Color = colorLED.Red;
      LeLED_e_Mode =  modeLED.Null;
    }
    else if (VeLED_e_ActnClaw == clawLED.PanelLtch) {
      LeLED_e_Color = colorLED.Blue;
      LeLED_e_Mode =  modeLED.Null;
    }
    else if (VeLED_e_ActnClaw == clawLED.ClawExtd) {
      LeLED_e_Color = colorLED.Red;
      LeLED_e_Mode =  modeLED.Null;
    }
    else if (VeLED_e_ActnClaw == clawLED.ClawRtct) {
      LeLED_e_Color = colorLED.Green;
      LeLED_e_Mode =  modeLED.Null;
    }
    else {
      LeLED_e_Color = colorLED.Off;
      LeLED_e_Mode =  modeLED.Null;
    } 


    /*******************************/
    /* Map LED Mode                */
    /*******************************/
    switch (LeLED_e_Mode)
      {
      case Chase:
        {
          LeLED_Cnt_CodeMode = ledModeChase;
          break;  
        }

        case Checkerboard:
        {
          LeLED_Cnt_CodeMode = ledModeCheckerboard;
          break;  
        }  

        case Breath:
        {
          LeLED_Cnt_CodeMode = ledModeBreath;
          break;  
        }  

        case Null:
        default:
        {
          LeLED_Cnt_CodeMode = ledNull;
          ;
          break;  
        }

      }

    /*******************************/
    /* Map LED Color               */
    /*******************************/
    switch (LeLED_e_Color) {
      case Red:
        {
        LeLED_Cnt_CodeColor = ledRed;
        break;
        }
        case Green:
        {
        LeLED_Cnt_CodeColor = ledGreen;
        break;
        }
        case Blue:
        {
        LeLED_Cnt_CodeColor = ledBlue;
        break;
         }
        case Yellow:
        {
        LeLED_Cnt_CodeColor = ledYellow;
        break;
        }
        case Rainbow:
        {
        LeLED_Cnt_CodeColor = ledRainbow;
        break;
        }
        case RainbowCycle:
        {
        LeLED_Cnt_CodeColor = ledRainbowCycle;
        break;
          }
        case Visualize:
          {
          LeLED_Cnt_CodeColor = ledVisualize;
          break;
          }
        case Off:
        default:
          {
          LeLED_Cnt_CodeColor = ledOff;
          break;
          }
      }

      VeLED_Cnt_CodeColor = LeLED_Cnt_CodeColor;
      VeLED_Cnt_CodeMode = LeLED_Cnt_CodeMode;    
        
      if (VeLED_Cnt_CodeMode != ledOff) {
        setMode(VeLED_Cnt_CodeColor, VeLED_Cnt_CodeMode);
      }
      else {
        setLEDs(VeLED_Cnt_CodeColor);
      }

    }



  private void setLEDs(int color){
    ledArduino.write(0x08, color);
  }

  private void setMode(int color, int mode){
    ledArduino.write(0x08, mode);
    ledArduino.write(0x08, color);
  }




  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new CC_LEDCntrl());
  }
}
