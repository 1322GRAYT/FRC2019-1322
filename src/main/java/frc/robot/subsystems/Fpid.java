/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.commands.CC_DrvVsnTgtDsbl;
import frc.robot.calibrations.K_PID;



/**
 * Class: Fpid - Performs P-I-F (No D) Control of the Rotation of the Robot
 * via Drive System Motors based on the Angular Position Feedback of the
 * Vision System based on the Limelight CrossHair Target Calibration vs
 * Current Target Image CrossHair.
 */
public class Fpid extends Subsystem {
	private Timer VePID_t_TgtCondTmr = new Timer();
	
	// Variable Declarations
	boolean VePID_b_RotEnbl;           // (boolean)
	boolean VePID_b_PstnErrWithInDB;   // (boolean)
	boolean VePID_b_TgtCondCmplt;      // (boolean)
	double  VePID_Deg_PstnAct;         // (degrees)
    double  VePID_Deg_PstnDsrd;	       // (degrees)
    double  VePID_Deg_PstnErr;         // (degrees)
    double  VePID_Deg_PstnErrAccum;    // (degrees)
    double  VePID_Pct_FdFwdCorr;       // (percent)
    double  VePID_Pct_PropCorr;        // (percent)
    double  VePID_Pct_IntglCorr;       // (percent)
    double  VePID_Pct_PwrCmnd;         // (percent power)
    double  VePID_r_PwrCmndNorm;       // (normalized power)
    double  VePID_t_TrckOnTgtTm;       // (seconds)
    double  VePID_t_TrckOnTgtTmMax;    // (seconds)
    
    
    /**********************************************/
    /* Public Interface Definitions        */
    /**********************************************/
    
    /** Method: resetPID_TgtCondTmr - Resets the Drive Rotation
     * Control Profile Timer at the beginning of a Drive Segments.  */ 
    public void resetPID_TgtCondTmr() {
    	VePID_t_TgtCondTmr.reset();
    }

    /** Method: setPID_Deg_RotPstnTgt - Interface to Set the Vision Target
      * Position Angle for the Robot Tracking PID Controller. (degrees)
      * (+ degrees ClockWise, - degrees Counter-Clockwise)
      *  @param1: Drive Target Tracking PID Enable Select (boolean)	
      *  @param2: Drive Target Tracking System Desired Target Angle (degrees: double) */	
    public void setPID_Deg_RotPstnTgt(boolean RotSysEnbl,
								      double  RotPstnTgt) {
		VePID_b_RotEnbl = RotSysEnbl;
    	VePID_Deg_PstnDsrd = RotPstnTgt;
    }  
    
    /** Method: getPID_r_RotCmndNorm - Interface to Get the Drive System Motor 
	  * Normalized Power Command for the Robot Rotate Control (-1 to 1).
      *  @return: Robot Rotate Motor Driver Normalized Power Command (-1 to 1: double) */	
    public double getPID_r_RotCmndNorm() {
    	return VePID_r_PwrCmndNorm;
    }

    /** Method: GetPID_b_TgtCondCmplt - Interface to Get the Robot Rotate PI
      * Control System Target Acquired Condition Complete Indication. (boolean)
      *  @return: Robot Rotate Target Condition Complete (boolen) */	
    public boolean GetPID_b_TgtCondCmplt() {
    	return VePID_b_TgtCondCmplt;
    }

    
    /** Method: mngPID_InitCntrl - Interface to Reset the Drive System
	  * Rotate PID Controller, i.e. initialize PID variables.  */ 
    public void mngPID_InitCntrl() {
        rstPID_Cntrlr();
    }
   

    // Below is for SmartDash Display   

    public double getPID_Deg_PstnAct() {
    	return VePID_Deg_PstnAct;
    }

    public boolean getPID_b_PstnErrWithInDB() {
		return VePID_b_PstnErrWithInDB;
	}

    public double getPID_Deg_PstnErr() {
    	return VePID_Deg_PstnErr;
    }

    public double getPID_Deg_ErrAccum() {
    	return VePID_Deg_PstnErrAccum;
    }

    public double getPIDRot_Pct_FdFwdTerm() {
    	return VePID_Pct_FdFwdCorr;
    }    
    
    public double getPID_Pct_PropTerm() {
    	return VePID_Pct_PropCorr;
    }

    public double getPID_Pct_IntglTerm() {
    	return VePID_Pct_IntglCorr;
    }
    
    public double getPID_Pct_PIDCmndPct() {
    	return VePID_Pct_PwrCmnd;
    }
  
    // Above is for SmartDash Display   


    /******************************************************/
    /* Manage Rotate PID Control Subsystem Scheduler Task  */
    /******************************************************/
	
    /** Method: managePIDRotate - Scheduler Function for the Periodic
     * Drive System Rotate Control PI Control System.  */ 
   public void managePIDRotate() {

	   VePID_Deg_PstnAct = Robot.VISION.getVSN_Deg_LL_TgtAngX();

   	 if (VePID_b_RotEnbl == true) {
	     VePID_Deg_PstnErr = calcPID_ErrSig(VePID_Deg_PstnDsrd, VePID_Deg_PstnAct,K_PID.KePID_Deg_PosErrDB);
       VePID_b_PstnErrWithInDB = dtrmnPID_ErrInDB(VePID_Deg_PstnErr, K_PID.KePID_Deg_PosErrDB); 
		   VePID_Deg_PstnErrAccum = calcPID_ErrAccum(VePID_Deg_PstnErrAccum, VePID_Deg_PstnErr, K_PID.KePID_Deg_IntglErrDsblMin);
		   VePID_Pct_FdFwdCorr = calcPID_FdFwdTerm(VePID_Deg_PstnErr);
		   VePID_Pct_PropCorr = calcPID_PropTerm(VePID_Deg_PstnErr, K_PID.KePID_K_PropGx, K_PID.KePID_Pct_PropCorrMax);
		   VePID_Pct_IntglCorr = calcPID_IntglTerm(VePID_Deg_PstnErrAccum, K_PID.KePID_K_IntglGx, K_PID.KePID_Pct_IntglCorrMax);
		   VePID_Pct_PwrCmnd = calcPID_TotCorr(VePID_Pct_FdFwdCorr, VePID_Pct_PropCorr, VePID_Pct_IntglCorr);
		   VePID_r_PwrCmndNorm = VePID_Pct_PwrCmnd/100;
		   VePID_b_TgtCondCmplt = dtrmnPID_TgtCondMet(VePID_b_PstnErrWithInDB, VePID_t_TgtCondTmr, K_PID.KePID_t_PstnTgtSyncMetThrsh);
   	 }
   	 else {  // (PIDEnbl == false)
   	   rstPID_Cntrlr();  
   	 }
   } 	


  /****************************************************/
  /*  Vision Closed Loop Error Calculations           */
  /****************************************************/	 
   
    /**********************************************/
    /* Internal Class Methods                     */
    /**********************************************/
   
	  /** Method: calcPID_ErrSig - Calculates Vision X-Axis Error in units of
	   * angular degrees, taking into account a symmetrical error dead-band
	   * around the zero-error point.
       * @param2: SetPoint -   Controller Target Set Point Value (double)
	   * @param2: ProcessVal - Current Input Angle along the X-Dimension (double)
	   * @param3: ThrshDB -    Dead-Band Threshold (double)
	   * @return: ErrSig -     Dead-Band Adjusted Error Value (double)  */
		 private double calcPID_ErrSig(double  SetPoint,
		 													  	 double  ProcessVal,
		 														   double  ThrshDB) {
		double  ErrSigRaw, ErrSig;
		
		/* Assumption Pos Angle indicates Robot facing Left of Target and must adjust Right,
		   will have Neg Error Signal. */
		
    	ErrSigRaw = (SetPoint - ProcessVal);

		if (ErrSigRaw >= 0) {
		  if (ErrSigRaw > ThrshDB) {
			ErrSig = ErrSigRaw - ThrshDB;
		  }
		  else {
			ErrSig = 0;
		  }   
		}
		else {  /* ErrSigRaw < 0) */
		  if (ErrSigRaw < -ThrshDB) {
			ErrSig = ErrSigRaw + ThrshDB;
		  }
		  else {
			ErrSig = 0;
		  }
		}
  
  return ErrSig;
  } 


  /** Method: dtrmnPID_ErrInDB - Determines if Controller Error
	* is within the targeted Dead-Band.  Used when when rotating
	* the robot to execute a turn when determining if the
	* desired target position is being attained.
	* @param1: Controller Signal Error Value (double)
	* @param2: Controller Actual Feedback Process Value (double)
	* @return: Indication whether the Error is within the Deadband (double)  */
	private boolean dtrmnPID_ErrInDB(double ErrSig,
								     double ThrshDB) {
	  boolean ErrInDB = false;
  
	  if ((ErrSig >= 0) && (ErrSig <= ThrshDB))
		ErrInDB = true;
	  else if ((ErrSig < 0.0) && (ErrSig >= -ThrshDB))
		ErrInDB = true;
  
	  return ErrInDB;
	}


  /** Method: calcPID_ErrAccum - Calculate the error accumulation
	* signal for use by the Integral Controller.
	* @param1: Controller Accumulated Error Signal (double)
	* @param2: Controller Error Signal (double)
	* @param3: Min Error Signal Thresh Above which Error will not
	*          be accumulated. (double)
	* @return: Updated Controller Accumulated Error Signal (double)  */
	private double calcPID_ErrAccum(double ErrAccum,
								    double ErrSig,
								    double ErrDsblThrshMin) {
	  double  ErrAccumTemp;
	  double  ErrSigAbs;
	  boolean SignFlipRst = false;
  
	  ErrSigAbs = Math.abs(ErrSig);
  
	  if ((ErrAccum > 0) && (ErrSig < 0)) {
		SignFlipRst = true;
	  } 
	  else if ((ErrAccum < 0) && (ErrSig > 0)) {
		SignFlipRst = true;
	  }
  
	  if(SignFlipRst == true) {
		ErrAccumTemp = (double)0.0;
	  }
	  else if (ErrSigAbs >= (double)ErrDsblThrshMin) {
		ErrAccumTemp = ErrAccum;
	  }
	  else {
		// (SignFlipRst == false) 
		ErrAccumTemp = ErrAccum + ErrSig;  
	  }	    
  
	  return ErrAccumTemp;  	  
	}
  
     
     /** Method: calcPID_FdFwdTerm - Calculate the Feed-Forward Correction Term.
      * @return: Feed-Forward Correction Term (double)  */
     private double calcPID_FdFwdTerm(double ErrSig) {
     	 float ErrAxis;   // scalar
    	 double FF_Corr;  // percent power
     	
     	ErrAxis = Robot.TBLLIB.AxisPieceWiseLinear_int((float)ErrSig,
     			                                       K_PID.KnPID_Deg_FdFwdErrAxis,
     			                                       (int)10);
     	
     	FF_Corr = Robot.TBLLIB.XY_Lookup_flt(K_PID.KtPID_Pct_FdFwdCorr,
     			                             ErrAxis,
     		   	                             (int)10);
     	
     	if (FF_Corr < 0.0) {
     		FF_Corr = 0.0;
     	} else if (FF_Corr > 100.0) {
     		FF_Corr = 100.0;;
     	}

     	return FF_Corr;
     }
     
	
    /** Method: calcPID_PropTerm - Calculate and Limit
      * the Controller Proportional Correction Term.
      * @param1: Controller Error Signal (double)
      * @param2: Controller Proportional Gain (double)
      * @param3: Proportional Correction Maximum Limit (double)
      * @return: Proportional Correction Term (double)  */
     private double calcPID_PropTerm(double ErrSig,
    		                         float  PropGx,
                                     float  CorrLimMax) {
     	double P_Corr;  // percent power
     	
     	P_Corr = (double)PropGx * ErrSig;
     	
     	if (P_Corr > (double)CorrLimMax) {
     		P_Corr = (double)CorrLimMax;
     	}
     	else if (P_Corr < (double)(-CorrLimMax)) {
     		P_Corr = (double)(-CorrLimMax);
     	}
     	return P_Corr;
     }

     
     /** Method: calcPID_IntglTerm - Calculate and Limit
       * the Controller Integral Correction Term.
       * @param1: Controller Accumulated Error Signal (double)
       * @param2: Controller Integral Gain (double)
       * @param3: Proportional Correction Maximum Limit (double)
       * @return: Integral Correction Term (double)  */
       private double calcPID_IntglTerm(double ErrAccum,
    		                            float  IntglGx,
    		                            float  CorrLimMax) {
     	 double I_Corr;     	
     	
     	 I_Corr = (double)IntglGx * ErrAccum;
     	     	
    	 if (I_Corr > CorrLimMax)
    		    I_Corr = CorrLimMax;

    	 else if (I_Corr < -CorrLimMax)
                I_Corr = -CorrLimMax;

     	 return I_Corr;
     }

     
     /** Method: calcPIDTotCorr - Calculate Total Correction which includes Proportional,
       * Integral Controller Correction, and any Feed Forward Correction Term.
       * @param1: Feed-Forward Correction Term - Percent Correction (double)
       * @param2: Proportional Correction Term - Percent Correction (double)
       * @param3: Integral Correction Term - Percent Correction (double)
       * @return: Total P-I Correction Term - Percent Correction (double)  */
     private double calcPID_TotCorr(double  FdFwdTerm,
    		                        double  PropTerm,
    		                        double  IntglTerm) {
     	double CmndPct; // %
     	
     	CmndPct = FdFwdTerm + PropTerm + IntglTerm;
 	    
 	    if (CmndPct > 100) {
 	      CmndPct = 100;
 	    }
 	    else if (CmndPct < 0) {
 	      CmndPct = 0;
 	    }

	  return CmndPct;
 
 	  }
     

     /** Method: dtrmnPID_TgtCondMet - Determine that the Target Condition
       * has been met for the minimum amount of time to be considered
       * satisfied. 
       * @param1: Indication of basic target condition has been met (boolean)
       * @param2: Condition Timer (double)
       * @param3: Target Condition Time Threshold (float)
       * @return: Indication that target condition time threshold has been met (boolean)  */
     private boolean dtrmnPID_TgtCondMet(boolean CondMet,
                                         Timer   CondTmr,
                                         float   CondMetThrsh) {
    	 boolean TgtCondCmptd = false;
    	
    	 if (CondMet == false)  { // Error outside Target DB
    		 CondTmr.reset(); 
    	 }
    	 else {
    		 /* Do Nothing - Inside Target DB - Free-Running Timer */
    	 }

    	 
    	 if (CondTmr.get() >= CondMetThrsh) {
    		 TgtCondCmptd = true; 
    	 }
    	 
    	 return TgtCondCmptd;
     }

     
     /** Method: dtrmnTmeOnCourse - Determine the amount of time that the Drive
       * Course Heading System has been on course. 
       * @param1: Indication of basic target condition has been met (boolean)
       * @param2: Condition Timer (double)
       * @return: Indication that target condition time threshold has been met (boolean)  */
    private double dtrmnDrvOnCourse(boolean CondMet,
   		                            Timer   CondTmr) {
   	
        double OnTgtTm;	
    	
   	    if (CondMet == false) { // Error outside Target DB
   		     CondTmr.reset(); 
   	    }
   	    else  {
   		     /* Do Nothing - Insided Target DB - Free-Running Timer */
   	    }
   	    	
   	    OnTgtTm = CondTmr.get();
   	 
   	    return OnTgtTm;
    }
     
     
     
     /** Method: rstPID_Cntrlr - YadaYada
      *  @param:  input info	(units)
      *  @return: output info (units) */	
     private void rstPID_Cntrlr() {
		   VePID_Deg_PstnAct = 0.0;
		   VePID_Deg_PstnDsrd = 0.0;
		   VePID_t_TgtCondTmr.reset();
       VePID_Deg_PstnErr = 0.0;
       VePID_b_PstnErrWithInDB = false;
		   VePID_Deg_PstnErrAccum = 0.0;
		   VePID_Pct_FdFwdCorr = 0.0;
		   VePID_Pct_PropCorr = 0.0;
		   VePID_Pct_IntglCorr = 0.0;
		   VePID_Pct_PwrCmnd =  0.0;
		   VePID_r_PwrCmndNorm =  0.0;
		   VePID_b_TgtCondCmplt = false;
		   VePID_t_TrckOnTgtTm = 0.0;
		   VePID_t_TrckOnTgtTmMax = 0.0;
     }
	      
	
    public void initDefaultCommand() {
      setDefaultCommand(new CC_DrvVsnTgtDsbl());
    }
}

