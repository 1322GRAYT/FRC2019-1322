/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;


public class TblLib extends Subsystem {
	
    private static final int MIN_INT =  0x8000;
    private static final int MAX_INT =  0x7FFF;	
    
    
	/** Method: AxisPieceWiseLinear_int - This function will return an rescaled
	  * axis index value from a Piece-Wise Linear tunable axis of type Integer.
	  * @param1: Axis Input Value in engineering units (float)
	  * @param2: Axis Array Object Reference (reference to array of ints)
	  * @param3: Table Size (int)
	  * @return: Rescaled Axis Output value in a normalized index value (float) */
	public float AxisPieceWiseLinear_int(float  InpVal,
	                                     int [] AxisArray,
	                                     int    TblSize)
	  {
	  int    InpValInt;
      int    TblSegs;
	  int    ArrayIdx;
	  float  InterpFrac;
	  float  AxisOutVal;

	  if (TblSize > 2) TblSegs = TblSize - 1;
	  else TblSegs = 1;

	  // Convert input value from float to int
	  if (InpVal <= (float)MIN_INT)
	    {
		InpValInt = MIN_INT;
	    }
	  else if (InpVal >= (float)MAX_INT)
	    {
		InpValInt = MAX_INT;
	    }
	  else
	    {
		InpValInt = (int)InpVal;
	    }

      /* determine where the input value is in the array based on the calibrated
       * cell values, and linearly interpolate between cells if necessary. */
	  if (InpValInt <= AxisArray[0])
	      {
		  AxisOutVal = (float)0.0;
	      }
	  else if (InpValInt >= AxisArray[TblSegs])
	      {
		  AxisOutVal = (float)1.0;
	      }
	  else
	      {
	      for (ArrayIdx = 1;
	    	   InpValInt >= AxisArray[ArrayIdx];
	    	   ArrayIdx++)
	        {}

	      ArrayIdx--;

	      InterpFrac = InterpCoefFrac(InpVal,
	                                  ((float)(AxisArray[ArrayIdx])),
	                                  ((float)(AxisArray[ArrayIdx + 1])));

	      AxisOutVal = ((float)ArrayIdx + InterpFrac) / (float)TblSegs;
	      }

	  return AxisOutVal;
	  }

	
	/** Method: AxisPieceWiseLinear_flt - This function will return an rescaled
	  * axis index value from a Piece-Wise Linear tunable axis of type float.
	  * @param1: Axis Input Value in engineering units (float)
	  * @param2: Axis Array Object Reference (reference to array of floats)
	  * @param3: Table Size (int)
	  * @return: Rescaled Axis Output value in a normalized index value (float) */	
	public float AxisPieceWiseLinear_flt(float  InpVal,
			                             float [] AxisArray,
			                             int    TblSize)
	  {
      int    TblSegs;
	  int    ArrayIdx;
	  float  InterpFrac;
	  float  AxisOutVal;

	  if (TblSize > 2) TblSegs = TblSize - 1;
	  else TblSegs = 1;

	  if (InpVal <= AxisArray[0])
	    {
		AxisOutVal = (float)0.0;
	    }
	  else if (InpVal >= AxisArray[TblSegs])
	    {
		AxisOutVal = (float)1.0;
	    }
	  else
	    {
	    for (ArrayIdx = 1;
	    	 InpVal >= AxisArray[ArrayIdx];
	    	 ArrayIdx++)
	    {}

	    ArrayIdx--;


	    InterpFrac = InterpCoefFrac(InpVal,
	    		                    AxisArray[ArrayIdx],
	    		                    AxisArray[ArrayIdx + 1]);

	    AxisOutVal = ((float)ArrayIdx + InterpFrac) / (float)TblSegs;
	    }

	  
	  return AxisOutVal;
	  }

	
	/** Method: AxisLinear_flt - This function will return an interpolated
	 * linear scalar value between two axis limit values (LwrRef and UprRef).
     * @param1: Axis Input Value in engineering units (float)
     * @param2: Axis Lower Bound Reference Value in engineering units (float)
     * @param3: Axis Upper Bound Reference Value in engineering units (float)
	 * @return: Rescaled Axis Output value in a normalized index value (float) */	
	public float AxisLinear_flt(float InpVal,
	                            float LwrRef,
	                            float UprRef)
	  {
	  float AxisOutVal;      

	  if (InpVal <= LwrRef)
	    {	  
		AxisOutVal = (float)0.0;
	    }	  
	  else if (InpVal >= UprRef)
        {
		AxisOutVal = (float)1.0;
        }
	  else
	    {
		AxisOutVal = InterpCoefFrac(InpVal, LwrRef, UprRef);
	    }

	  return AxisOutVal;
	  }
	
	
	/** Method: InterpCoefFrac - This function will return the Fractional
	 *  Interpolation Coefficient value of a floating point value between two 
	 *  floating point reference values.
     * @param1: Interpolation Input Value (float)
     * @param2: Lower Reference value to Interpolate Between (float)
     * @param3: Upper Reference value to Interpolate Between (float)
	 * @return: Fractional Interpolated Output Value (float) */	
	public float InterpCoefFrac(float Inp,
	                            float LwrRef,
	                            float UprRef)
	  {
	  return ((Inp - LwrRef)/(UprRef - LwrRef));
	  }

	
	/** Method: InterpCoef - This function will return the Fractional
	 *  Interpolation Coefficient value of a floating point value between two 
	 *  floating point reference values.	
     * @param1: Interpolation Input Value (float)
     * @param2: Lower Reference value to Interpolate Between (float)
     * @param3: Upper Reference value to Interpolate Between (float)
	 * @return: Fractional Interpolated Output Value (float) */
//   todo: lambda expression - need to learn how to add a label to reference the expression	
/* 	 (Inp, LwrRef, UprRev) -> ((Inp - LwrRef)/(UprRef - LwrRef)); */

	
	/** Method: LinearInterp_flt - This function will return the Linear Interpolation
	 *  of two given points (Lower and Upper Reference) given a fractional coefficient
	 *  factor scalar (0 - 1) 
     * @param1: Lower Reference value to Interpolate Between (float)
     * @param2: Upper Reference value to Interpolate Between (float)
     * @param3: Fractional Factor Scalar Input Value (float)
	 * @return: Interpolated Output Value (float) */	
	public float LinearInterp_flt(float LwrRef,
	                              float UprRef,
	                              float FractCoef)
	  {
	  return ((FractCoef * (UprRef - LwrRef)) + LwrRef);
	  }

	
	/******************************************************************************
	 * Function:     fltLimitTableIndex_flt
	 * Description:  This function limits table index values between 0 and 1.
	 *****************************************************************************/
	/** Method: LmtTblIdx_Flt - This function limits table index values between
	  * 0 and 1 (0 - 1). 
     * @param1: Axis Table Index Value (float)
	 * @return: Interpolated Output Value (float) */	
	public float LmtTblIdx_Flt(float TblIdx)
	  {
	  if (TblIdx < (float)0.0) TblIdx = (float)0.0;
	  else if (TblIdx > (float)1.0) TblIdx = (float)1.0;
	  return TblIdx;
	  }	
		
	
	/** Method: XY_Lookup_flt - This function will return an interpolated
	 * table look-up value based on a rescaled normalized axis index value.
     * @param1: Look-Up Table Array Object Reference (reference to array of floats)
     * @param2: Normalized Axis Index Input value (float)
     * @param3: Table Size (int)
	 * @return: Table Look-Up Output value in engineering units (float) */	
	public float XY_Lookup_flt(float [] TblArray,
	                           float  AxisInpIdx,
	                           int    TblSize)
	  {
	  int    TblSegs;
	  int    ArrayIdx;
	  float  InterpFrac;
	  float  TblOutVal;

	  if (TblSize > 2) TblSegs = TblSize - 1;
	  else TblSegs = 1;	  
	  
	  /* Limit table index values between 0 and 1. */
      AxisInpIdx = LmtTblIdx_Flt(AxisInpIdx);
	  
	  if (AxisInpIdx <= (float)0.0)
	      {
		  TblOutVal = TblArray[0];
	      }
	  else if (AxisInpIdx >= (float)1.0)
	      {
		  TblOutVal = TblArray[TblSegs];
	      }
	  else
	      {
	      /* ArrayIdx     = floor(AxisInpIdx * (LeSize - 1)) */
		  InterpFrac   = AxisInpIdx * (float)TblSegs;
		  ArrayIdx = (int)InterpFrac;

	      /* LeInterpFrac     = (LeTableIndex * (LeSize - 1)) - LeArrayIndex */
	      InterpFrac  -= (float)ArrayIdx;

	      /* LeY = LeLower + (LeInterpFrac * (LeUpper - LeLower)) */
	      TblOutVal = LinearInterp_flt(TblArray[ArrayIdx],
	    		                       TblArray[ArrayIdx + 1],
	    		                       InterpFrac);
	      }

	  return TblOutVal;
	  }


	/** Method: XYZ_Lookup_flt - This function will return an interpolated
	 * table look-up value based on a rescaled normalized axis index value.
     * @param1: Look-Up Table Array Object Reference (reference to 3-D array of floats)
     * @param2: Normalized Y-Axis Index Input value (float)
     * @param3: Normalized X-Axis Index Input value (float)
     * @param4: Table Y-Axis Size (int)
     * @param5: Table X-Axis Size (int)
	 * @return: Table Look-Up Output value in engineering units (float) */	
	public float XYZ_Lookup_flt(float [][] TblArray,
                                float AxisInpIdxY,
                                float AxisInpIdxX,
                                int   TblSizeY,
                                int   TblSizeX)
	{
        int   TblSegsX;
        int   TblSegsY;
  	    int   ArrayIdxX;
	    int   ArrayIdxY;
        int   BaseArrayIdx;
  	    float InterpFracX;
	    float InterpFracY;
	    float RefLwrY;
	    float RefUprY;
        float TblOutVal;

        
  	    if (TblSizeX > 2) TblSegsX = TblSizeX - 1;
  	    else TblSegsX = 1;	  
        
  	    if (TblSizeY > 2) TblSegsY = TblSizeY - 1;
  	    else TblSegsY = 1;	  
		

		/* Limit table index values between 0 and 1. */
		AxisInpIdxX = LmtTblIdx_Flt(AxisInpIdxX);
		AxisInpIdxY = LmtTblIdx_Flt(AxisInpIdxY);
		
		/* LeArrayIndexX     = floor(LeTableIndexX * (LeSizeX - 1)) */
		InterpFracX   = AxisInpIdxX * TblSegsX;
		ArrayIdxX = (int)InterpFracX;
		
		/* LeInterpFracX     = (LeTableIndexX * (LeSizeX - 1)) - LeArrayIndexX */
		InterpFracX  -= (float)ArrayIdxX;
		
		/* LeArrayIndexY     = floor(LeTableIndexY * (LeSizeY - 1)) */
		InterpFracY   = AxisInpIdxY * TblSegsY;
		ArrayIdxY = (int)InterpFracY;
		
		/* LeInterpFracY     = (LeTableIndexY * (LeSizeY - 1)) - LeArrayIndexY */
		InterpFracY  -= (float)ArrayIdxY;
		
		
		if (AxisInpIdxY < (float)1.0)
		{
			if (AxisInpIdxX < (float)1.0)
			{
				/* IF: LeTableIndexY < 1, LeTableIndexX < 1 */
				
				/* LeLowerY = LeLowerX1 + (LeInterpFracX * (LeUpperX1 - LeLowerX1)) */
					RefLwrY = LinearInterp_flt(TblArray[ArrayIdxX][ArrayIdxY],
						                       TblArray[ArrayIdxX + 1][ArrayIdxY],
						                       InterpFracX);
				
				/* LeUpperY = LeLowerX2 + (LeInterpFracX * (LeUpperX2 - LeLowerX2)) */
					RefUprY = LinearInterp_flt(TblArray[ArrayIdxX][ArrayIdxY + 1],
						                       TblArray[ArrayIdxX + 1][ArrayIdxY + 1],
						                       InterpFracX);
				
				/* LeZ = LeLowerY + (LeInterpFracY * (LeUpperY - LeLowerY)) */
				TblOutVal = LinearInterp_flt(RefLwrY, RefUprY, InterpFracY);
			}
			else
			{
				/* IF: LeTableIndexY < 1, LeTableIndexX = 1 */
				/* LeLowerY = LeLowerY1 */
				/* LeUpperY = LeLowerY2 */
				/* LeZ      = LeLowerY + (LeInterpFracY * (LeUpperY - LeLowerY)) */
				TblOutVal = LinearInterp_flt(TblArray[TblSegsX][ArrayIdxY],
					                         TblArray[TblSegsX][ArrayIdxY + 1],
					                         InterpFracY);
			}		
		}
		else
		{
			if (AxisInpIdxX < (float)1.0)
			{
				/* IF: LeTableIndexY = 1, LeTableIndexX < 1 */
				/* LeLowerY = LeLowerX1 */
				/* LeUpperY = LeUpperX1 */
				/* LeZ      = LeLowerY + (LeInterpFracY * (LeUpperY - LeLowerY)) */
				TblOutVal = LinearInterp_flt(TblArray[ArrayIdxX][TblSegsY],
						                     TblArray[ArrayIdxX + 1][TblSegsY],
			                                 InterpFracX);
			}
			else
			{
				/* IF: LeTableIndexY = 1, LeTableIndexX = 1 */
				/* LeZ = LeLower[XHi][YHi] */
				TblOutVal = TblArray[TblSegsX][TblSegsY];
			}
		}		
		return TblOutVal;
	}
	
	
    /** Method: RateLimOnInc - This function will rate limit an increase
      *  delta command based on a maximum limit threshold in delta
      *  units per loop.
      * @param1: Interpolation Input Value (float)
      * @param2: Lower Reference value to Interpolate Between (float)
      * @param3: Upper Reference value to Interpolate Between (float)
	  * @return: Fractional Interpolated Output Value (float) */	
	 public static double RateLimOnInc(double ValRaw,
			                           double ValLim,
			                           float  DeltLimMax) {
	     double DeltUpd;
	     double ValTemp;
	     double ValLimNew;
	     boolean LimPosInc;
	     boolean LimNegInc;
	     	     

	     // Determine if PosIncLim, NegIncLim, or No Limiting
	     if (ValLim >= 0) {
	    	 // ValLim is Positive
	    	 if (ValRaw < 0.0) {
	    		 // Sign Flip to Negative
	    		 ValLim = 0.0;
	    		 LimNegInc = true;
	    		 LimPosInc = false;
	    	 } else if (ValRaw > ValLim) {
	    		 LimPosInc = true;
	    		 LimNegInc = false;
	         } else {
	    		 // (ValRaw <= ValLim) && ValRaw is not Negative
	    		 // Decreasing Positive, No Limit Applied.
	    		 LimPosInc = false;
	    		 LimNegInc = false;
	    	 }
	     } else {
	    	 //  ValLim is Negative
	    	 if (ValRaw > 0.0) {
	    		 // Sign Flip to Positive
	    		 ValLim = 0.0;
	    		 LimPosInc = true;	    		 
	    		 LimNegInc = false;
	    	 } else if (ValRaw < ValLim) {
	    		 LimNegInc = true;	    		 
	    		 LimPosInc = false;
	    	 } else {
	    		 // (ValRaw >= ValLim) && ValRaw is not Positive
	    		 // Increasing Negative, No Limit Applied.
	    		 LimPosInc = false;
	    		 LimNegInc = false;
	    	 }	    	 
	     }

	     
	     // Apply Delta Limit
	     DeltUpd = Math.abs(ValRaw) - Math.abs(ValLim);
	     if (DeltUpd > DeltLimMax) {
		     DeltUpd = DeltLimMax;
	     }
	     
	     // Add Delta to Last Loop Value
         if (LimPosInc == true) {
    	     ValTemp = ValLim + DeltUpd;        
         } else if (LimNegInc == true) {
    	     ValTemp = ValLim - DeltUpd;              	 
	     } else {
	    	 // No Limiting;
	    	 ValTemp = ValRaw;
	     }
         
         ValLimNew = ValTemp; 

	     return ValLimNew;
     }
     

     
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    

}
